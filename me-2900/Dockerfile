FROM docker.synergeticon.com/natscpp

# install deps
RUN apt update && apt install -y \
    libgio2.0-cil-dev \
    libglib2.0-dev \
    intltool \
    gtk-doc-tools \
    libxml2-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/
RUN git clone https://github.com/AravisProject/aravis.git \
    && cd aravis \
    && ./autogen.sh \
    && ./configure \
    && make \
    && make install \
    && rm -r /opt/aravis \
    && ldconfig

RUN mkdir /driver \
    && cp /usr/lib/x86_64-linux-gnu/glib-2.0/include/glibconfig.h /usr/include/glib-2.0/glibconfig.h
ADD driver.cc /driver/
ADD GetProfilesCallback.h /driver/
ADD device_properties.dat /driver/
ADD Makefile /driver/

ADD libllt /driver/
ADD libllt /usr/include/libllt
ADD libmescan /driver/
ADD libmescan /usr/include/libmescan

ADD entities.pb.cc /driver/
ADD entities.pb.h /driver/

ADD libllt/libllt.so.1.0 /usr/local/lib/
ADD libmescan/libmescan.so.1.0 /usr/local/lib/

# Make driver
WORKDIR /driver/
RUN ldconfig && make
ENTRYPOINT ["./driver"]