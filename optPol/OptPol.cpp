/*################################################################################
# MIT License
# 
# Copyright (c) 2021 Sebastian Pilarski and Slawomir Pilarski
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
################################################################################*/

#include <iostream>
#include <vector>
#include <cmath>
#include <cassert>
#include <thread>
#include <atomic>
#include <chrono>
#include <algorithm>
#include <boost/python.hpp>

typedef double kfloat; // so we can experiment with numeric accuracy

inline int ceil_dK( int x, int K ) { return (x+K-1)/K; } // ceil(x/K)

struct bits {
    std::vector<uint32_t> w;
    long loc ( long num ) { return (num >> 5) + (num & 0x1F); }
    bits(long s) { w.resize(loc(s),0); } 
    void set( long l)       {        w[l>>5] |= (0x01 << ((int)l & 0x1F)); }
    bool get( long l) const { return w[l>>5] &  (0x01 << ((int)l & 0x1F)); }
};


//--------------------------------------------------------------------------------
//  Structure to keep s_i f_i and n_i configurations (see paper)
//--------------------------------------------------------------------------------
struct s_sf {
  int s;   // number of succeses
  int f;   // number of failures
  int n;   // number of pulls
  int a;   // arm id  
  s_sf(int as, int af, int aa = 0 ) : s(as), f(af), n(as+af), a(aa) {};
  s_sf() : s(0), f(0), n(0) {};
  bool operator<(const s_sf& o) const {return n > o.n || (n == o.n && s > o.s); } // we want decreasing sort
};

typedef std::vector<s_sf> t_vsf;     // vector of s/f/n for eacch arm, a configuration

void print_vsf(const t_vsf& vsf ) {  // use to peek inside a configuration
  int t = 0;
  for ( auto& e : vsf ) { t += e.n; std::cout << e.s << "  " << e.f << "  " << e.n << ";  ";  }
  std::cout << "  t: " << t << std::endl;
}

inline void sort( t_vsf& v) { // our indexing requires sorted configurations
  std::stable_sort(v.begin(),v.end());  
} 

//--------------------------------------------------------------------------------
//  recursive version of function g -- see section on k-arms (as in paper)
//--------------------------------------------------------------------------------
long g( int k, int n, int lim ) {
  if ( k == 1 ) return n+1;
  long sum = 0;
  for ( int i = ceil_dK(n,k); i <= lim; i++ ) {
    sum += (i+1)* g(k-1,n-i,std::min(i,n-i));  
  }
  return sum;
}

//--------------------------------------------------------------------------------
//  recursive version of funciton offset -- seee section on k-arms
//--------------------------------------------------------------------------------
long offset( const t_vsf& vsf, int location = 0 ) {
    int k = (int) vsf.size() - location;
    int n = vsf[location].n; //if ( n == 0 ) return 0;
    int s = vsf[location].s;
    int t = 0; for ( int i = location; i < k + location; i++ ) t += vsf[i].n;
    long sum  = g(k,t,n-1);
         sum += s * g(k-1,t-n,std::min(n,t-n)); 
         sum += k == 2 ? vsf[location+1].s : offset(vsf,location+1);
    return sum;
};

//--------------------------------------------------------------------------------
//  Non-recursive implementation of index -- MUUUCH FASTER! (Lookup tables)
//--------------------------------------------------------------------------------

struct indxK {
  static constexpr int MH = 300;  // Max Time Horizon
  static constexpr int MK = 12;   // Max k in k-arm
  long   LT[MK+1][MH+1][MH+1];    // Lookup Table
  long   OP[MK+1][MH+1];          // Optimal Policy Storage Size
  
  indxK() { init(); }
  
  void init() {
    // int Lookup Table
    for ( int k = 0; k <= MK; k++ ) {             // for each k-arm
      for ( int n = 0; n <= MH; n++ ) {           // for each n pulls
        for ( int lim = 0; lim <= MH; lim++ ) {   // for each sum limit
            if ( k == 0 ) LT[k][n][lim] = 0;   else
            if ( k == 1 ) LT[k][n][lim] = n+1; else
            if ( n == 0 ) LT[k][n][lim] = 1;   else {
              long sum = 0;
              for ( int i = ceil_dK(n,k); i <= lim; i++ ) {
                  sum += (i+1) * LT[k-1][n-i][std::min(i,n-i)];
              }
              LT[k][n][lim] = sum;
            }
        }
      }
    } 
    // init Optimal Policy Storage Size
    for ( int k = 0; k <= MK; k++ ) {             // for each k-arm
      for ( int h = 0; h <= MH; h++ ) {           // for each horizon
          if ( k == 0 || h == 0 ) OP[k][h] = 0; else {
              if ( k == 1 ) OP[k][h] = h;         // one arm = one option to pull
              else          OP[k][h] = OP[k][h-1] + LT[k][h-1][h-1];
          }
      }
    }
  }
  
  long opss     ( int k, int t ) const { return OP[k][t];    }
  long step_size( int k, int t ) const { return LT[k][t][t]; }
  
  long offset( const t_vsf& vsf ) const {  // from the start of storage for t
    long sum = 0;
    for ( int start = 0; ; ) {
      int  k   = (int) vsf.size() - start;
      int  t   = 0; for ( int i = start; i < k + start; i++ ) t += vsf[i].n;
      int  n   = vsf[start].n;
      int  s   = vsf[start].s;
           sum += n == 0 ? 0 : LT[k][t][n-1];
           sum += s * LT[(k-1)][t-n][std::min(n,t-n)];
           if ( k > 2 ) start++; 
           else {
             sum += vsf[start+1].s;
             break;
           }
    } 
    return sum;
  }
  
  long index( const t_vsf& vsf ) const {  // from start of storage for t = 0
     int t = 0; for ( auto& e : vsf ) t += e.n;
     return opss( vsf.size(), t ) + offset(vsf);
  }
};

//--------------------------------------------------------------------------------

indxK iK; // one copy will serve all experiments

//--------------------------------------------------------------------------------
//  Config Iterator Class - iterates through all configurations, 
//  foundation for multithreaded classes
//  Nore: by overloading process_vsf() various tasks can be accomplished
//--------------------------------------------------------------------------------

class cnfg_itr {
  public:
    int   K   = 0; // number k-arms
    long  ctr = 0; // configuration counter for each t
    long  tot = 0; // total configuration counter
    t_vsf vsf;     // configuration vector
  public:
    int   id    = 0; // for multithreading; thread ID
    int   thrds = 1; // number of threads
    t_vsf v;         // tmp vector 
    long  act   = 0; // counts thread calls to process_vsf; sanity check
  public:
               cnfg_itr(int k, int Id, int nth) : K(k), id(Id), thrds(nth) {;}
      virtual ~cnfg_itr() {;}
      virtual void process_vsf() {;} // process current vsf configuration
      
      long for_all_configs( int t ) { // for all time t configs do process_vsf
          vsf.resize(K); v.resize(K);
          ctr = 0; 
          //for_all_rec(K,t,t);
          for_all_nrec(K,t);
          return ctr;
      }
      
  private:
      // non-recursive version of for_all_t -- seems to be no real speed diff
      void for_all_nrec( int K, int T ) { // K-arms T-time_horizon
        act = 0;
        int From[K]; // Sum From index
        int To  [K]; // Sum To   index
        int tt  [K]; // Number of pulls left
        int nt  [K]; // current n_i  
        int st  [K]; // current s_i
        From[0] = ceil_dK( T,K);
        To  [0] = T;
        tt  [0] = T;
        nt  [0] = From[0];
        st  [0] = 0;
        for ( int loc = 0; loc < K; ) {
          int t = tt[loc];
          int n = nt[loc];
          int s = st[loc];
          if ( n > To[loc] ) {       // done with current loc ?
              if ( loc == 0 ) break; // we have fnished
              loc--; continue;
          } else {
              if ( s > n ) {         // done with current n ?
                nt[loc]++; 
                st[loc] = 0; continue;
              } else {
                st [loc]++; // still work to do for this loc 
                vsf[loc].s = s;
                vsf[loc].n = n;
                vsf[loc].f = n-s;
              }
          }
          int  k = K - loc;       // arms to finish config
          if ( k == 1 ) {         // last arm?
                vsf[loc].n = n; // -- not needed here
                for ( int s1 = 0; s1 <= n; s1++ ) {
                    vsf[loc].s = s1; vsf[loc].f = n-s1;
                    //print_vsf();
                    // code below assures thead safety of optimal policy computation
                    int  t    = 0; for (int l=0; l < K; l++) t += vsf[l].n;
                    long shrtctr = ctr + iK.opss(K,t);
                         shrtctr >>= 6;  // 2^6 = 64 == 8 *sizeof(long)
                    if ( (shrtctr % thrds ) == id ) { act++;
                           process_vsf(); // PROCESS IT HERE
                    }
                    ctr++;
                    tot++;
                }
                loc--;
          } else {
                From[loc+1] = ceil_dK(t-n,k-1); // arms ordered by n_i
                To  [loc+1] = std::min(n,t-n);  // arms ordered by n_i
                tt  [loc+1] = t - n;
                nt  [loc+1] = From[loc+1];
                st  [loc+1] = 0;
                loc++; continue;
          }
        }
      }

      // recursive version of for_all_t
      void for_all_rec( int k, int t, int lim, int loc = 0 ) {
          if ( k == 1 ) {
                int n1 = t;
                vsf[loc].n = n1;
                for ( int s1 = 0; s1 <= n1; s1++ ) {
                  vsf[loc].s = s1; vsf[loc].f = n1-s1;
                  //print_vsf();
                  // code below assures thead safety of optimal policy computation
                    int  t    = 0; for (int l=0; l < K; l++) t += vsf[l].n;
                    long shrtctr = ctr + iK.opss(K,t);
                         shrtctr >>= 6;  // 2^6 = 64 == 8 *sizeof(long)
                    if ( (shrtctr % thrds ) == id ) { act++;
                           process_vsf(); // PROCESS IT HERE
                    }
                  ctr++;
                  tot++;
                }
          } else {
            for ( int n0 = ceil_dK( t,k); n0 <= lim; n0++ ) {
              vsf[loc].n = n0;
              for ( int s0 = 0; s0 <= n0; s0++ ) {
                vsf[loc].s = s0; vsf[loc].f = n0-s0;
                for_all_rec(k-1, t-n0, std::min(n0,t-n0), loc+1 );
              }
            }
          }
      }
};

//--------------------------------------------------------------------------------
//  Test for Config Iterator Class - checks configurations vs their offset
//  This is an example of cnfg_itr usage
//--------------------------------------------------------------------------------

class cnfg_itr_test : public cnfg_itr {
  public:
      using cnfg_itr::cnfg_itr;             // test inherits consturctors
      virtual void process_vsf() override { // offset and ctr must agree
            long off = iK.offset(vsf);
            long idx = iK.index(vsf);
	    std::cout << "test: config_number = " << tot << " index = " << idx << " \tconfig = "; print_vsf(vsf);
            if ( ctr !=off ) std::cout << "Error: " << ctr << " -> " << off << std::endl;
            if ( tot !=idx ) std::cout << "Error: " << ctr << "  " << tot << " -> " << idx << std::endl;
      }
};


//--------------------------------------------------------------------------------
//  Config Iterator Class for expected value computation
//  This is another example of cnfg_itr usage
//--------------------------------------------------------------------------------

// priors for all arms
      int alf0 = 0;
      int bet0 = 0;
      
class cnfg_itr_expv : public cnfg_itr {
  public:
      std::vector<kfloat> *pR = 0; // pointer to ready data
      std::vector<kfloat> *pC = 0; // pointer to currently computed
      std::vector< std::vector<bool> >* pOd = 0; // optimal decision marks
      
      using cnfg_itr::cnfg_itr; // test inherits consturctors
      virtual void process_vsf() override { 
                // Note: it could be just one loop, but to find all best choices
                 long fI[K]; // fail index for t+1; for each arm
                 long sI[K]; // fail index for t+t; for each arm
                 for ( int l = 0; l < K; l++ ) {
                   std::copy(vsf.begin(),vsf.end(),v.begin());
                   v[l].n++; sort(v);
                   fI[l] = iK.offset(v); 
                   std::copy(vsf.begin(),vsf.end(),v.begin());
                   v[l].n++; v[l].s++; sort(v);
                   sI[l] = iK.offset(v);
                 }
                 kfloat e[K]; // expected value for t; for each arm
                 for ( int l = 0; l < K; l++ ) {
                   const auto&  sf = vsf[l];
                   const kfloat p  = (kfloat)(alf0+sf.s)/(alf0+bet0+sf.n);
                   e[l] = ((*pR)[sI[l]] + 1)*p +
                          ((*pR)[fI[l]] + 0)*(1-p);

                 }
                 int    best =   0;
                 kfloat maxe = e[0];
                 for ( int l = 1; l < K; l++ ) {
                   if ( maxe < e[l] ) { maxe = e[l]; best = l; } else
                   if ( maxe == e[l] ) { /* another max */ }
                 }
                 (*pC)[ctr] = maxe;
                 // now mark optimal decisions
                 if ( pOd ) {
                     int  t    = 0; for (int l=0; l < K; l++) t += vsf[l].n;
                     long indx = ctr + iK.opss(K,t);
                     for ( int l = 1; l < K; l++ ) {
                       if ( maxe == e[l] ) { (*pOd)[l][indx] = true; }
                     }
                 }
      }
};


//--------------------------------------------------------------------------------
//  Config Iterator Class for reachability computation 
//       -- assumes optimal policy is provided
//--------------------------------------------------------------------------------

class cnfg_itr_reach : public cnfg_itr {
  public:
      std::vector< std::vector<bool> >* pOd = 0; // optimal decision marks
      std::vector<bool>*                pRe = 0; // reachable marks
      int  H         = 0;
      long reachable = 0;
      long greedy    = 0;

      using cnfg_itr::cnfg_itr; // test inherits consturctors
      virtual void process_vsf() override {
                int  t    = 0; for (int l=0; l < K; l++) t += vsf[l].n;
                long idx = ctr + iK.opss(K,t);
                if ( !(*pRe)[idx] && t > 0 ) { 
                   return;
                }
                reachable++;
                // find first optimal decision 
                int  best = 0;
                for ( int l = 0; l < K; l++ ) {
                    if ( (*pOd)[l][idx] ) { best = l; break; }
                }
                // check if decision is greedy
                kfloat g[K];
                for ( int l = 0; l < K; l++ ) {
                    g[l] = (kfloat)vsf[l].s / vsf[l].n;
                }
                int greq = 0; // number of greater-equal
                for ( int l = 0; l < K; l++ ) {
                    greq += g[l] >= g[best];
                }
                if ( greq == 1 ) greedy++;
                
                //if ( t >= iK.opss(K,H-1) ) return; // if last t
                
                std::copy(vsf.begin(),vsf.end(),v.begin());
                v[best].n++; sort(v);
                (*pRe)[iK.index(v)] = true;; 
                std::copy(vsf.begin(),vsf.end(),v.begin());
                v[best].n++; v[best].s++; sort(v);
                (*pRe)[iK.index(v)] = true;; 
      }
};


//--------------------------------------------------------------------------------
//  Class to run multiple threads - base class for various computations
//--------------------------------------------------------------------------------


// Single thread funciton; needs its own cnfg_itr, 
// Note: direction of commputation is selectable (t may increase or decrease)
void run_thrd( cnfg_itr* p, std::atomic<bool>& flg, int H, const int dir) { 
    assert( dir==-1 || dir==1 );
    for ( int i = 0; i < H; i++ ) {
        while ( flg.load() ); // flg == 0 means start working
        int t = dir == 1 ? i : H - 1 -i;
        auto cntr = p->for_all_configs(t);
        flg  = true;
        // std::cout << p->id;
    }
    //std::cout << "DONE" << std::endl;
}

class mrunner {
  protected:
    const int K      = 2; // K-arms
    const int H      = 5; // Horizon
    const int nthrds = 1; // number of threads
    static constexpr   int   MaxThrds = 32; // MAX number of threads
    std::atomic<bool> flgs[MaxThrds];     // to synchronize after t is done 
    cnfg_itr*         itrs[MaxThrds];     // defined in derived classes
    
  public:
    mrunner(int k, int h, int nth ) : K(k), H(h), nthrds(nth) {
        for ( auto& f : flgs ) f = false;
        for ( auto& i : itrs ) i = 0;
    }
    virtual ~mrunner() {;}
    
    virtual void sync() {;}  // do something before another t is prorocessed

    void mrun(int dir) {
        long act = 0;
        long sum = 0;
        std::thread myThreads[nthrds];
        for (int i=0; i < nthrds; i++) myThreads[i] = std::thread( run_thrd, itrs[i],std::ref(flgs[i]),H,dir);
        for (int i=0; i < H; i++ ) {
          std::cout << "    t: " << ( dir > 0 ? i : H - i ) << std::endl; 
          sum = 0;
          SLEEP:
          // put sleep here
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
          int rdy = 0; for ( int i = 0; i < nthrds; i++ ) rdy += flgs[i].load();
          if ( rdy < nthrds ) goto SLEEP;
          sync();
          for ( int i=0; i < nthrds; i++) act     += itrs[i]->act;
          //std::cout << "t: " << i << " DONE; sum: " << sum << " \t threads: " << nthrds<< " act: " << act << std::endl;
          for ( int i=0; i < nthrds; i++) flgs[i]  = false;
        }
        for (int i=0; i < nthrds; i++) myThreads[i].join();
        //for (int i=0; i < nthrds; i++) sum += ctrs[i];
        //for (int i=0; i < nthrds; i++) act += itrs[i]->act;
        //for ( auto c : ctrs ) std::cout << c << "  ";
        //std::cout << "H: " << H << " DONE; sum: " << sum << " \t threads: " << nthrds<< " act: " << act << std::endl;
    }
    
    void test() {
        for ( int i = 0; i < nthrds; i++ ) { assert(!itrs[i]); itrs[i] = new cnfg_itr_test(K,i,nthrds); } 
        mrun(1);
        for ( int i = 0; i < nthrds; i++ ) { delete(itrs[i]); itrs[i] = 0; }
    }
};

//--------------------------------------------------------------------------------
//  multi-threaded expected value computation
//  This is an example how to use mrunner
//--------------------------------------------------------------------------------

class mrunnerExp : public mrunner {
    std::vector<kfloat> ssR; // step storage ReadOnly
    std::vector<kfloat> ssC; // step storage Change
  public:
   using mrunner::mrunner;
   virtual ~mrunnerExp() {;}
   
   virtual void sync() override { ssR.swap(ssC); }

   void compute() {
      ssR.resize(iK.step_size(K,H),0);
      ssC.resize(iK.step_size(K,H),0);
      for ( int i = 0; i < nthrds; i++ ) { assert(!itrs[i]); auto p = new cnfg_itr_expv(K,i,nthrds); p->pR = &ssR; p->pC = &ssC; itrs[i] = p; } 
      mrun(-1);
      for ( int i = 0; i < nthrds; i++ ) { delete(itrs[i]); itrs[i] = 0; }
      std::cout  << H << " EXPECTED: " << ssR[0]/H << std::endl;
   }
};

//--------------------------------------------------------------------------------
//  multi-threaded reachability computation
//  This is another example of how to use mrunner
//    Note: a part of reachability computation is run as a single thread
//    std::vector<bool> used here is not thread-safe (many bool values per word);
//--------------------------------------------------------------------------------

class mrunnerReach : public mrunner {
    std::vector<kfloat> ssR; // step storage Read
    std::vector<kfloat> ssC; // step storage Change
    std::vector< std::vector<bool > > odm; // optimal decision mark 
    std::vector<bool>                 rea; // reachable
    

  public:
   using mrunner::mrunner;
   virtual ~mrunnerReach() {;}
   
   virtual void sync() override { ssR.swap(ssC); }
   void compute() {
      ssR.resize(iK.step_size(K,H),0);
      ssC.resize(iK.step_size(K,H),0);
      odm.resize(K);
      for( int i=0; i<K; i++) odm[i].resize(iK.opss(K,H));
      rea.resize(iK.opss(K,H+1));
      for ( int i = 0; i < nthrds; i++ ) { assert(!itrs[i]); auto p = new cnfg_itr_expv(K,i,nthrds); p->pR = &ssR; p->pC = &ssC; p->pOd = &odm; itrs[i] = p; } 
      mrun(-1);  // commpute "backwards" to find optimal policy marks
      for ( int i = 0; i < nthrds; i++ ) { delete(itrs[i]); itrs[i] = 0; }
      //std::cout  << H << " EXPECTED: " << ssR[0]/H << std::endl;
      
      const_cast<int&>(nthrds) = 1; //  To avoid non-atomic behavior of cnfg_itr_reach (see code)
      for ( int i = 0; i < nthrds; i++ ) { assert(!itrs[i]); auto p = new cnfg_itr_reach(K,i,nthrds);  p->pOd = &odm; p->pRe = &rea; p->H = H; itrs[i] = p; } 
      mrun(1); // compute "forward" to mark reachable configurations
      long reachable = 0; long greedy = 0; long act = 0; long ctr = 0;
#if 0
      for ( int i = 0; i < nthrds; i++ ) { 
         auto p = static_cast<cnfg_itr_reach*>(itrs[i]);
         std::cout << "\t" << p->ctr;
      }
      for ( int i = 0; i < nthrds; i++ ) { 
         auto p = static_cast<cnfg_itr_reach*>(itrs[i]);
         std::cout << "\t" << p->reachable;
      }
      std::cout << std::endl;
#endif
      for ( int i = 0; i < nthrds; i++ ) { 
         auto p = static_cast<cnfg_itr_reach*>(itrs[i]);
         reachable += p->reachable;
         greedy    += p->greedy;
         act       += p->act;
         ctr       += p->ctr;
      }
      auto sum = iK.opss(K,H);
      std::cout << H << " reach: " << (double)reachable/sum << "\treach-gr/reach: " << (double)(reachable-greedy)/reachable << "\treach-gr/all " << (double)(reachable-greedy)/sum << " all: " << sum << std::endl;

      for ( int i = 0; i < nthrds; i++ ) { delete(itrs[i]); itrs[i] = 0; }
   }
};


//--------------------------------------------------------------------------------
//  multi-threaded optimal policy computation
//  This is another example of how to use mrunner
//    Note: THIS IS A CRIPPLED VERSION; USE ONLY ONE THREAD !!!
//    std::vector<bool> used here is not thread-safe (many bool values per word);
//--------------------------------------------------------------------------------

static std::vector< std::vector<bool > > odm__; // optimal decision mark 
static    t_vsf v__;   // temp for sorting

class mrunnerOptPol : public mrunner {
    std::vector<kfloat> ssR; // step storage Read
    std::vector<kfloat> ssC; // step storage Change
    

  public:
   using mrunner::mrunner;
   virtual ~mrunnerOptPol() {;}
   
   virtual void sync() override { ssR.swap(ssC); }
   void compute() {
      ssR.resize(iK.step_size(K,H),0);
      ssC.resize(iK.step_size(K,H),0);
      odm__.resize(0); // free most of previously allocated storage (if any)
      odm__.resize(K);
      v__  .resize(K);
      for( int i=0; i<K; i++) odm__[i].resize(iK.opss(K,H));
      for ( int i = 0; i < nthrds; i++ ) { assert(!itrs[i]); auto p = new cnfg_itr_expv(K,i,nthrds); p->pR = &ssR; p->pC = &ssC; p->pOd = &odm__; itrs[i] = p; } 
      mrun(-1);  // commpute "backwards" to find optimal policy marks
      for ( int i = 0; i < nthrds; i++ ) { delete(itrs[i]); itrs[i] = 0; }
      //std::cout  << H << " EXPECTED: " << ssR[0]/H << std::endl;
      
      for ( int i = 0; i < nthrds; i++ ) { delete(itrs[i]); itrs[i] = 0; }
   }
   static void clear() {
      odm__.resize(0);
      v__  .resize(0);
      alf0 = 0;
      bet0 = 0;
   }
   static int best_arm( const t_vsf& vsf ) {
     std::copy(vsf.begin(),vsf.end(),v__.begin());
     for ( int i = 0; i < (int)odm__.size(); i++ ) v__[i].a = i;
     sort( v__ );
     long idx = iK.index( v__ );  
     // find first optimal decision 
     int  best = 0;
     for ( int l = 0; l < (int)odm__.size(); l++ ) {
         if ( odm__[l][idx] ) { best = l; break; }
     }
     //std::cout << "best found " << best << " -> " << v__[best].a << std::endl;
     return v__[best].a;
   }
   static int n_arms() { return (int)odm__.size(); }
};


//--------------------------------------------------------------------------------
//   functions for python api
//--------------------------------------------------------------------------------

long index2( int s0, int f0, int s1, int f1 ) {
    int t = s0 + f0 + s1 + f1;
    if ( t > indxK::MH ) {
      std::cout << "ERROR: parameters exceeded max time horizon of " << indxK::MH << std::endl;
      return 0;
    }
    t_vsf vsf;
    vsf.emplace_back(s0,f0);
    vsf.emplace_back(s1,f1);
    sort( vsf );
    return iK.index(vsf);
}

long index3( int s0, int f0, int s1, int f1, int s2, int f2 ) {
    int t = s0 + f0 + s1 + f1 + s2 + f2;
    if ( t > indxK::MH ) {
      std::cout << "ERROR: parameters exceeded max time horizon of " << indxK::MH << std::endl;
      return 0;
    }
    t_vsf vsf;
    vsf.emplace_back(s0,f0);
    vsf.emplace_back(s1,f1);
    vsf.emplace_back(s2,f2);
    sort( vsf );
    return iK.index(vsf);
}

void test( int k, int h, int nth ) {
    mrunner mr(k,h,nth);
    mr.test();
}
void expected( int k, int h, int alf, int bet, int nth ) {
    // Function computes normalized expected cumulative reward for optimal policy.
    // Note: multiply result by h to get true expected cumulative reward.
    // k - number of arms
    // h - time horizon
    // Beta(alf,bet) is prior
    // nth - number of threads
        if ( k > indxK::MK ) {
          std::cout << "ERROR: parameters exceeded " << indxK::MH << " arms" << std::endl;
          return;
        }
        if ( h > indxK::MH ) {
          std::cout << "ERROR: parameters exceeded max time horizon of " << indxK::MH << std::endl;
          return;
        }
        mrunnerExp mr(k,h,nth);  
        // set the prior
        auto tmp_alf = alf0;
        auto tmp_bet = bet0;
        alf0 = alf;
        bet0 = bet;
        mr.compute();
        alf0 = tmp_alf;
        bet0 = tmp_bet;
}

void reachable( int k, int h, int alf, int bet, int nthrds ) {
    // Function computes fraction of reachable entries in the optimal policy table.
    // k - number of arms
    // h - time horizon
    // Beta(alf,bet) is prior
        if ( k > indxK::MK ) {
          std::cout << "ERROR: parameters exceeded " << indxK::MH << " arms" << std::endl;
          return;
        }
        if ( h > indxK::MH ) {
          std::cout << "ERROR: parameters exceeded max time horizon of " << indxK::MH << std::endl;
          return;
        }
        mrunnerReach mr(k,h,nthrds); // USE ONLY ONE THREAD HERE!!!
        // set the prior
        auto tmp_alf = alf0;
        auto tmp_bet = bet0;
        alf0 = alf;
        bet0 = bet;
        mr.compute();
        alf0 = tmp_alf;
        bet0 = tmp_bet;
}

void OptPol_compute( int k, int h, int alf, int bet, int nthrds ) {
    // Function computes the optimal policy table.
    // k - number of arms
    // h - time horizon
    // Beta(alf,bet) is prior
        if ( k > indxK::MK ) {
          std::cout << "ERROR: parameters exceeded " << indxK::MH << " arms" << std::endl;
          return;
        }
        if ( h > indxK::MH ) {
          std::cout << "ERROR: parameters exceeded max time horizon of " << indxK::MH << std::endl;
          return;
        }
        mrunnerOptPol mr(k,h,nthrds); // USE ONLY ONE THREAD HERE!!!
        // set the prior
        alf0 = alf;
        bet0 = bet;
        mr.compute();
}

void OptPol_clear() { mrunnerOptPol::clear();  }   // clear policy table
int  OptPol_narms() { return mrunnerOptPol::n_arms(); }   // policy table: number of arms
int  OptPol_alf  () { return alf0;             }   // policy table: Beta prior parameter
int  OptPol_bet  () { return bet0;             }   // policy table: Beta prior parameter

int OptPol_best_arm2( int s0, int f0, int s1, int f1 ) {
    // Function returns best arm according to the optimal policy table
        int t = s0 + f0 + s1 + f1; // + s2 + f2;
        if ( mrunnerOptPol::n_arms() != 2 ) {
          std::cout << "ERROR: parameters expected for " << mrunnerOptPol::n_arms() << " arms" << indxK::MH << std::endl;
          return 0;
        }
        if ( t > indxK::MH ) {
          std::cout << "ERROR: parameters exceeded max time horizon of " << indxK::MH << std::endl;
          return 0;
        }
        t_vsf vsf;
        vsf.emplace_back(s0,f0);
        vsf.emplace_back(s1,f1);
        //vsf.emplace_back(s2,f2);
        return mrunnerOptPol::best_arm(vsf);
}

int OptPol_best_arm3( int s0, int f0, int s1, int f1, int s2, int f2 ) {
    // Function returns best arm according to the optimal policy
        int t = s0 + f0 + s1 + f1 + s2 + f2;
        if ( mrunnerOptPol::n_arms() != 3 ) {
          std::cout << "ERROR: parameters expected for " << mrunnerOptPol::n_arms() << " arms" << indxK::MH << std::endl;
          return 0;
        }
        if ( t > indxK::MH ) {
          std::cout << "ERROR: parameters exceeded max time horizon of " << indxK::MH << std::endl;
          return 0;
        }
        t_vsf vsf;
        vsf.emplace_back(s0,f0);
        vsf.emplace_back(s1,f1);
        vsf.emplace_back(s2,f2);
        return mrunnerOptPol::best_arm(vsf);
}

BOOST_PYTHON_MODULE(OptPol) {
    boost::python::def("index2",    index2           ); 
    boost::python::def("index3",    index3           ); 
    boost::python::def("test",      test             ); 
    boost::python::def("expected",  expected         ); 
    boost::python::def("reachable", reachable        ); 
    boost::python::def("compute",   OptPol_compute   ); 
    boost::python::def("clear",     OptPol_clear     ); 
    boost::python::def("n_arms",    OptPol_narms     ); 
    boost::python::def("alf",       OptPol_alf       ); 
    boost::python::def("bet",       OptPol_bet       ); 
    boost::python::def("best_arm2", OptPol_best_arm2 ); 
    boost::python::def("best_arm3", OptPol_best_arm3 ); 
}


//--------------------------------------------------------------------------------
//  main()
//--------------------------------------------------------------------------------

#if 0
int main(int argc, char **argv) {
    expected(3,50,3,7,16);
    return 0;
    std::cout << "Hello, world!" << std::endl;
    int K = 3;  // number of runs
    int H = 50; // MA
    for ( int k = K; k > 1; k-- ) {
      std::cout << "\n--------------- k: " << k << std::endl;
      for ( int t = H; t <= H; t++ ){
        //mrunnerReach mr(k,t,1); // uncomment to run reachability
        mrunnerExp mr(k,t,16);   // uncomment to run
        // set the prior
        alf0 = 3;
        bet0 = 7;
        // mrExp.test(); // uncomment to run test()
        mr.compute();

      }
    }
    return 0;
}

#endif
