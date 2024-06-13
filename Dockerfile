FROM mxwilliam/mxck:mxck-melodic-base-l4t-35.2.1 

RUN python3 -m pip install --upgrade pip \
    && python3 -m pip install \
    statistics \
    pycryptodome

RUN apt update \
 && apt install --yes \
 python3.8 \
 python3-pip \
 && update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.6 1 \
 && update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.8 2

COPY ./ros_entrypoint.sh /ros_entrypoint.sh
RUN echo 'source /ros_entrypoint.sh' >> ~/.bashrc

COPY ./autorun.sh /
ENTRYPOINT ["./autorun.sh"]
CMD ["false"]

WORKDIR ./melodic_ws