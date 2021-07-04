<!-- PROJECT LOGO -->
<br>
<p align="center">
  <a href="https://github.com/enriquetomasmb/bci-driving">
    <img src="experiments/bci-driving.png" alt="BCI" width="943" height="462">
  </a>
  <h3 align="center">Brain-Computer Interface Project</h3>
  <br>
  <h3 align="center">Cognitive experiment</h3>

  <p align="center">
    Detection of attention/distraction while driving on the road.
    <br>
  </p>
</p>

## About the project

<a href="https://um.es">
  <img src="experiments/umu.jpg" alt="BCI" width="195" height="50">
</a>
<br>
This project is part of the End of Master Project (New Technologies in Computing) at the University of Murcia.

### Prerequisites

* Python version 3.7 or more, pyenv recommended
* pip3

### Installation

1. Clone the repo
```sh
git clone https://github.com/enriquetomasmb/bci-driving.git
```
With ```--branch develop``` you will get the developing branch.
```sh
git clone --branch develop https://github.com/enriquetomasmb/bci-driving.git
```
2. Change to project directory
3. (Optional) Create your virtual environment, and activate it (you can also use ```conda``` to create the virtual environment)
```sh
python -m venv env

source env/bin/activate  # Linux/Mac
env/Scripts/activate  # Windows
```
3. Install required packages
```sh
pip3 install -r requirements.txt
```

## Usage

### ```bci_driving.py```

### ```bci_npc.py```

### ```bci_realtime.py```

### ```analysis/signal_analysis.ipynb```


## Tools

* [Versatile EEG - Bitbrain](https://www.bitbrain.com/neurotechnology-products/semi-dry-eeg/versatile-eeg) -  EEG device for real-time recording.
* [Python](https://www.python.org/) - Python and the libraries for the creation of the experiment and EEG signal synchronization


## Roadmap

See the [open issues](https://github.com/enriquetomasmb/bci-driving/issues) for a list of proposed features (as well as known issues).


## Author

* **Enrique Tomás Martínez Beltrán** - [Website](https://enriquetomasmb.com) - [LinkedIn](https://www.linkedin.com/in/enrique-tomas/)


## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details

