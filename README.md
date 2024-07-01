<!-- PROJECT LOGO -->
<br>
<p align="center">
  <h3 align="center">SAFECAR: A Brain-Computer Interface and intelligent framework to detect drivers’ distractions</h3>
  <br>
</p>

## About the project

<a href="https://um.es">
  <img src="experiments/umu.jpg" alt="BCI" width="195" height="50">
</a>
<br>
This project is part of my End of Master's Project at the University of Murcia.

This repository contains a related code for the paper *[SAFECAR: A Brain-Computer Interface and intelligent framework to detect drivers’ distractions](https://doi.org/10.1016/j.eswa.2022.117402)*

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

## Tools

* [Versatile EEG - Bitbrain](https://www.bitbrain.com/neurotechnology-products/semi-dry-eeg/versatile-eeg) -  EEG device for real-time recording.
* [Python](https://www.python.org/) - Python and the libraries for the creation of the experiment and EEG signal synchronization


## Roadmap

See the [open issues](https://github.com/enriquetomasmb/bci-driving/issues) for a list of proposed features (as well as known issues).


## Author

* **Enrique Tomás Martínez Beltrán** - [Website](https://enriquetomasmb.com) - [LinkedIn](https://www.linkedin.com/in/enrique-tomas/)


## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details


## Citation

If you use this repository, please cite our [paper](https://doi.org/10.1016/j.eswa.2022.117402)

```
@article{MartinezBeltran-safecar-2022,
	title		= {{SAFECAR: A Brain-Computer Interface and intelligent framework to detect drivers' distractions}},
	author		= {Mart{\'i}nez Beltr{\'a}n, Enrique Tom{\'a}s and Quiles P{\'e}rez, Mario and L{\'o}pez Bernal, Sergio and Mart{\'i}nez P{\'e}rez, Gregorio and Huertas Celdr{\'a}n, Alberto},
	year		= 2022,
	journal		= {Expert Systems with Applications},
	volume		= 203,
	pages		= 117402,
	doi		= {10.1016/j.eswa.2022.117402},
	issn		= {0957--4174},
	keywords	= {Brain-Computer Interfaces, Electroencephalographic signal, Cognitive state, Distraction detection, Framework, Machine Learning},
	abstract	= {As recently reported by the World Health Organization (WHO), the high use of intelligent devices such as smartphones, multimedia systems, or billboards causes an increase in distraction and, consequently, fatal accidents while driving. The use of EEG-based Brain–Computer Interfaces (BCIs) has been proposed as a promising way to detect distractions. However, existing solutions are not well suited for driving scenarios. They do not consider complementary data sources, such as contextual data, nor guarantee realistic scenarios with real-time communications between components. This work proposes an automatic framework for detecting distractions using BCIs and a realistic driving simulator. The framework employs different supervised Machine Learning (ML)-based models on classifying the different types of distractions using Electroencephalography (EEG) and contextual driving data collected by car sensors, such as line crossings or objects detection. This framework has been evaluated using a driving scenario without distractions and a similar one where visual and cognitive distractions are generated for ten subjects. The proposed framework achieved 83.9\% F1-score with a binary model and 73\% with a multiclass model using EEG, improving 7\% in binary classification and 8\% in multi-class classification by incorporating contextual driving into the training dataset. Finally, the results were confirmed by a neurophysiological study, which revealed significantly higher voltage in selective attention and multitasking.},
}
```

