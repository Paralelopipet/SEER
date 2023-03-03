# SEER

SEER - Stabilizing Energy Efficient Controller Manipulators using Reinforcement Learning

Built by Damjan Denic, Martin Graf, Nav Leelarathna, and Sepand Dyanatkar

## Setup

1. Clone the repository inclusive submodules `git clone --recurse-submodules git@github.com:Paralelopipet/SEER.git`
2. Create conda environment `conda env create -f environment.yml`
    - note: this will take a while (3 to 4 coffees)
3. Activate environment `conda activate l32_seer`
4. Install gym package `pip install --editable pybullet_multigoal_gym`
5. Install seer package `pip install --editable .`

## Test

Run `pytest`

## Run Simulation

`python pybullet_multigoal_gym/pybullet_multigoal_gym/examples/kuka_tip_over.py`

## Debug Simulation in VSCode

Press `F5` (launch configuration `Tip Over Simulation`)

### Codestyle

- Please use 4 space indentation.
