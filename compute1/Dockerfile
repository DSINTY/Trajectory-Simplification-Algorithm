FROM python:3.10
RUN apt-get install curl -y
RUN pip install Fred-Frechet
COPY . /
RUN git clone https://github.com/DanielSinTY/traj-sim-data.git