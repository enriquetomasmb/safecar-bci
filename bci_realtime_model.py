
# Brain-Computer Interface Project
# Cognitve experiment
# Enrique Tomás Martínez Beltrán - https://enriquetomasmb.com

import matplotlib.pyplot as plt
import mne

from mne.datasets import sample
from mne.io import read_raw_fif
from mne.viz import plot_events

from mne_realtime import LSLClient, MockLSLStream, RtEpochs

import threading
import multiprocessing
import time
import sys
from termcolor import colored

import pandas as pd
from joblib import load



def start_recording(terminate_event, host, wait_max, q, qev, qevd, path):
    print(colored("Starting start_recording", "green"))
    datos_input = []

    n_epochs = 0
    with LSLClient(host=host, wait_max=wait_max) as client:
        client_info = client.get_measurement_info()
        print(colored(client_info, 'yellow'))
        sfreq = int(client_info['sfreq'])

        started = False # False
        pipeline = load(path + 'model.joblib')
        print(pipeline)

        while terminate_event.value:
            if (started or q.get()):
                started = True
                
                epoch = client.get_data_as_epoch(n_samples=sfreq)
                
                # Prediction
                # True/False -> Distraction/Non-Distraction
                # 0,51,52 -> Tipo de distraction
                pred = pipeline['model'].predict(epoch)
                print(colored(pred, "yellow"))

                datos_input.append(epoch)
                
                n_epochs += 1

        print(colored("EEGRecording closed", "red"))

