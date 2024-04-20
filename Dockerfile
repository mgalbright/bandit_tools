#Dockerfile included to simplify building & running optPol in standardized environment.
#
#Build a docker image named opt_mab with command:
#  docker build -t opt_mab ./
#Run the image with
#  docker run -it opt_mab
#Then test it with
#  python3 OptPol.py

FROM ubuntu:20.04

# prevent ubuntu from asking questions during build
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        g++ \
        libboost-all-dev \
        libboost-python-dev \
        python3 \
        python3-pip \
        vim && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

COPY . /app

RUN rm /app/optPol/OptPol.so && \
    chmod +x /app/optPol/build.me && \
    cd /app/optPol && bash build.me

WORKDIR /app

CMD ["bash"]
