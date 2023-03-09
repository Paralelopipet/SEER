FROM continuumio/miniconda3:latest

ADD ./environment.yml /seer/environment.yml

WORKDIR /seer
RUN conda env create -f environment.yml

ADD . /seer

SHELL ["conda", "run", "-n", "l32_seer", "/bin/bash", "-c"]

RUN pip3 install --editable pybullet_multigoal_gym && pip3 install --editable pybullet_multigoal_implementation && pip3 install --editable .

ENTRYPOINT [ "conda", "run", "--no-capture-output", "-n", "l32_seer", "python", "/seer/pybullet_multigoal_implementation/drl_implementation/examples/FetchReachHER.py" ]
