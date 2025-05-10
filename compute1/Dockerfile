FROM python:3.10
RUN apt-get install curl -y
RUN pip install Fred-Frechet
COPY . /
RUN git clone https://github.com/DSINTY/traj-sim-data.git
# ADD "https://api.github.com/repos/username/repo_name/commits?per_page=1" latest_commit
# RUN curl -sLO "https://github.com/username/repo_name/archive/main.zip" && unzip main.zip