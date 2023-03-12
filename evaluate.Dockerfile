FROM debian:buster-slim

RUN apt-get update
RUN apt-get install -y --no-install-recommends make build-essential libssl-dev zlib1g-dev libbz2-dev libreadline-dev libsqlite3-dev wget curl llvm libncurses5-dev xz-utils tk-dev libxml2-dev libxmlsec1-dev libffi-dev liblzma-dev
RUN apt-get install -y mecab-ipadic-utf8

ENV HOME="/root"

WORKDIR $HOME
RUN apt-get install -y git
RUN git clone --depth=1 https://github.com/pyenv/pyenv.git .pyenv

ENV PYENV_ROOT="$HOME/.pyenv"
ENV PATH="$PYENV_ROOT/shims:$PYENV_ROOT/bin:$PATH"
RUN pyenv install 3.8.6
RUN pyenv global 3.8.6


COPY requirements.txt /root/requirements.txt
RUN pip3 install -r requirements.txt

ADD . /root

RUN pip3 install --editable pybullet_multigoal_gym && pip3 install --editable pybullet_multigoal_implementation && pip3 install --editable .

# ENTRYPOINT [ "conda", "run", "--no-capture-output", "-n", "l32_seer", "python", "/seer/pybullet_multigoal_implementation/drl_implementation/examples/FetchReachHER.py" ]
ENTRYPOINT ["python", "/root/pybullet_multigoal_implementation/drl_implementation/examples/FetchReachHER.py"]

