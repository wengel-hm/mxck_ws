FROM mxwilliam/mxck:mxck-melodic-base-l4t-35.2.1 


RUN apt-get update \
 && apt-get install --yes \
    python3.8-dev \
    build-essential \
    python-dev \
    liblapack-dev \
    libblas-dev \
    libglpk-dev \
    libopenblas-dev \
    libsuitesparse-dev 

ENV CPPFLAGS="-I/usr/include/suitesparse"
    
RUN python3 -m pip install --upgrade pip \
    qpsolvers \
    transformations \
    cvxopt \
    clarabel \
    highspy

    
COPY ./ros_entrypoint.sh /ros_entrypoint.sh
RUN echo 'source /ros_entrypoint.sh' >> ~/.bashrc

COPY ./autorun.sh /
ENTRYPOINT ["./autorun.sh"]
CMD ["false"]

WORKDIR ./melodic_ws
