FROM ubuntu:20.04 AS build-env

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Tokyo

RUN apt-get update
RUN apt-get -y --no-install-recommends install build-essential
RUN apt-get -y --no-install-recommends install clang
RUN apt-get -y --no-install-recommends install cmake
RUN apt-get -y --no-install-recommends install python3.9-dev
RUN apt-get -y --no-install-recommends install python3-pip
RUN apt-get -y --no-install-recommends install swig
RUN pip3 install setuptools

WORKDIR /app
COPY . .

ENV BUILD_DIR=/app/build
RUN mkdir -p $BUILD_DIR
WORKDIR $BUILD_DIR

RUN cmake .. && \
    cmake --build . && \
    cp ../setup.py . && \
    python3 setup.py build_ext --inplace

RUN ls /app/build
