
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


def start_recording(host, wait_max, q, qev, path):
    print(colored("Starting start_recording", "green"))
    # Load a file to stream raw data
    #data_path = sample.data_path()
    #raw_fname = data_path + '/MEG/sample/sample_audvis_filt-0-40_raw.fif'
    #raw = read_raw_fif(raw_fname).crop(0, 30).load_data().pick('eeg')
    datos_input = []
    timestamps = []
    estimulos = []
    # For this example, let's use the mock LSL stream.
    try:
        n_epochs = 0
        with LSLClient(host=host, wait_max=wait_max) as client:
            client_info = client.get_measurement_info()
            print(colored(client_info, 'yellow'))
            sfreq = int(client_info['sfreq'])

            # plt.ion()
            # fig, ax = plt.subplots(figsize=(15, 8))

            started = True # False
            event = (0,0)
            while True:
                if (started or q.get()):
                    started = True
                    if not qev.empty():
                        event = qev.get()
                    
                    # plt.cla()
                    epoch, ts = client.get_data_as_epoch(n_samples=sfreq)
                    timestamps.extend(ts)

                    if event != (0,0):
                        print(colored("{} - Event {}".format(event[1], event[0]), 'green'))
                    epoch = mne.epochs.combine_event_ids(epoch, ['1'], {str(event[0]): 0})
                    epoch.selection = [n_epochs]

                    # ya se envían en uV, no hace falta hacer scaling
                    #df_epoch = epoch.to_data_frame(time_format=None, scalings=dict(eeg=1)) 
                    
                    datos_input.append(epoch)
                    
                    n_epochs += 1
                    event = (0,0)

                    #epoch.average().plot(axes=ax)
                    # fig.canvas.draw()
                    # fig.canvas.flush_events()
            
    finally:
        if datos_input:
            epochs_all = mne.concatenate_epochs(datos_input)
            epochs_all.save(path + 'test-epo.fif')
            #df_final = pd.concat(datos_input)
            #df_final.to_csv('out.csv', index=None)
        
        # Creamos df auxiliar con timestamps
        df_final = []
        for i, ep in enumerate(datos_input):
            df = ep.to_data_frame(time_format=None, scalings=dict(eeg=1))
            df['Timestamp'] = timestamps[i]
            df_final.append(df)

        df_final_out = pd.concat(df_final)
        df_final_out.to_csv(path + 'out.csv', index=None, date_format='%s.%f')

        print(colored("EEGRecording closed", "red"))

if __name__ == '__main__':

    # epochs = mne.read_epochs('test-epo.fif', preload=True)
    # epochs.plot(scalings='auto', block=True)
    

    queue = multiprocessing.Queue()
    qev = multiprocessing.Queue()
    start_recording('openbcigui', 60, queue, qev)
