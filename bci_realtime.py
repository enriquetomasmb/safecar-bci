
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

map_event_distraction = {
    0 : "Non-Distraction",
    51 : "Math",
    52 : "Box",
}

map_event = {
    0 : "Non-Event",
    11 : "Cross-Line",
    15 : "Red-TrafficLight",
    21 : "Car-Collision",
    22 : "Pedestrian-Collision",
    23 : "Building-Collision"
}


def start_recording(terminate_event, host, wait_max, q, qev, qevd, path):
    print(colored("Starting start_recording", "green"))
    # Load a file to stream raw data
    #data_path = sample.data_path()
    #raw_fname = data_path + '/MEG/sample/sample_audvis_filt-0-40_raw.fif'
    #raw = read_raw_fif(raw_fname).crop(0, 30).load_data().pick('eeg')
    datos_input = []
    timestamps = []
    event_by_epoch = []
    # For this example, let's use the mock LSL stream.
    n_epochs = 0
    with LSLClient(host=host, wait_max=wait_max) as client:
        client_info = client.get_measurement_info()
        print(colored(client_info, 'yellow'))
        sfreq = int(client_info['sfreq'])

        # plt.ion()
        # fig, ax = plt.subplots(figsize=(15, 8))

        started = False # False
        event = (0,0) # Extern events
        event_distraction = (0,0,0) # Math or Box

        last_event_distraction = (0,0,0)
        event_distraction_cont = 0

        while terminate_event.value:
            if (started or q.get()):
                started = True
                if not qev.empty():
                    event = qev.get()
                if not qevd.empty():
                    event_distraction = qevd.get()
                
                # plt.cla()
                epoch = client.get_data_as_epoch(n_samples=sfreq)
                # timestamps.extend(ts)
                # print(epoch.get_data())
                event_by_epoch.append(event)
                
                if event_distraction_cont != 0:
                    epoch = mne.epochs.combine_event_ids(epoch, ['1'], {map_event_distraction[int(last_event_distraction[0])] : int(last_event_distraction[0])})
                    epoch.selection = [n_epochs]
                    event_distraction_cont -= 1
                    if event_distraction_cont == 0:
                        event_distraction = (0,0,0)
                else:
                    if event_distraction != (0,0,0):
                        # Si se ha mostrado Math (5sec) = marcar esta Epoch y las 4 siguientes
                        # Si se ha mostrado Box (5sec) = marcaar esta Epoch y las 4 siguientes
                        #print(colored("{} - Event {}".format(event_distraction[1], event_distraction[0]), 'green'))
                        last_event_distraction = event_distraction
                        event_distraction_cont = event_distraction[1] # distraction duration
                        epoch = mne.epochs.combine_event_ids(epoch, ['1'], {map_event_distraction[int(last_event_distraction[0])] : int(last_event_distraction[0])})
                        epoch.selection = [n_epochs]
                        event_distraction_cont -= 1
                    else:
                        epoch = mne.epochs.combine_event_ids(epoch, ['1'], {map_event_distraction[int(event_distraction[0])] : int(event_distraction[0])})
                        epoch.selection = [n_epochs]

                # ya se envían en uV, no hace falta hacer scaling
                #df_epoch = epoch.to_data_frame(time_format=None, scalings=dict(eeg=1)) 
                
                datos_input.append(epoch)
                
                n_epochs += 1
                event = (0,0)
                event_distraction = (0,0,0)
                #print(time.time())
                #epoch.average().plot(axes=ax)
                # fig.canvas.draw()
                # fig.canvas.flush_events()

        if datos_input:
            epochs_all = mne.concatenate_epochs(datos_input)
            epochs_all.save(path + 'experiment-epo.fif')
            #df_final = pd.concat(datos_input)
            #df_final.to_csv('out.csv', index=None)

            print(event_by_epoch)
            print(len(event_by_epoch) == len(datos_input))
            # Creamos df auxiliar con timestamps
            df_final = []
            for i, ep in enumerate(datos_input):
                df = ep.to_data_frame(time_format=None, scalings=dict(eeg=1))
                #df['Timestamp'] = timestamps[i]
                df['event'] = event_by_epoch[i][0]
                df['event_str'] = map_event[int(event_by_epoch[i][0])]
                df_final.append(df)
                # Asociamos el evento al Epoch anterior, el anterior al que acabamos de meter en df_final
                if event_by_epoch[i][0] != 0:
                    df_final[-2]['event'] = df_final[-1]['event']

            df_final_out = pd.concat(df_final)
            df_final_out = df_final_out.rename(columns={'condition': 'distraction_str'})
            distraction_ids = []
            for i in df_final_out['distraction_str']:
                # print(list(map_event_distraction.values()).index(i))
                # print(map_event_distraction.keys())
                distraction_ids.append(list(map_event_distraction.keys())[list(map_event_distraction.values()).index(i)])

            df_final_out.insert(1,'distraction', distraction_ids)
            df_final_out.to_csv(path + 'out.csv', index=None, date_format='%s.%f')

        print(colored("EEGRecording closed", "red"))

    # finally:
    #     if datos_input:
    #         epochs_all = mne.concatenate_epochs(datos_input)
    #         epochs_all.save(path + 'experiment-epo.fif')
    #         #df_final = pd.concat(datos_input)
    #         #df_final.to_csv('out.csv', index=None)

    #         print(event_by_epoch)
    #         print(len(event_by_epoch) == len(datos_input))
    #         # Creamos df auxiliar con timestamps
    #         df_final = []
    #         for i, ep in enumerate(datos_input):
    #             df = ep.to_data_frame(time_format=None, scalings=dict(eeg=1))
    #             #df['Timestamp'] = timestamps[i]
    #             df['event'] = event_by_epoch[i][0]
    #             df['event_str'] = map_event[int(event_by_epoch[i][0])]
    #             df_final.append(df)
    #             # Asociamos el evento al Epoch anterior, el anterior al que acabamos de meter en df_final
    #             if event_by_epoch[i][0] != 0:
    #                 df_final[-2]['event'] = df_final[-1]['event']

    #         df_final_out = pd.concat(df_final)
    #         df_final_out = df_final_out.rename(columns={'condition': 'distraction'})
    #         df_final_out.to_csv(path + 'out.csv', index=None, date_format='%s.%f')

    #     print(colored("EEGRecording closed", "red"))

if __name__ == '__main__':

    # epochs = mne.read_epochs('test-epo.fif', preload=True)
    # epochs.plot(scalings='auto', block=True)
    

    queue = multiprocessing.Queue()
    qev = multiprocessing.Queue()
    qevd = multiprocessing.Queue()
    start_recording('openbcigui', 60, queue, qev, qevd, 'test')
