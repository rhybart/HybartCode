#!/usr/bin/env python3
# coding: utf-8

from scipy import signal

class EMGChannel:
    sf = 1000 #sampling freq (Hz)
    hf = 50 #highpass cutoff freq (Hz)
    lf = 6 #lowpass cutoff freq (Hz)
    window = 500 #sample window
    offset=-32678
    # try doing this instead of doing the offset values *3.3/ 65536;
    w = hf/(sf/2)
    wl = lf/(sf/2)
    
    
    b, a = signal.butter(2, w,'high') #highpass coefficients
    bl, al = signal.butter(2, wl, 'low') #lowpass coefficients
    
    def __init__(self, name):
        self.name = name
        self.rawdata = [] #will store all values
        self.windowedrawdata = [] #just the most recent 500 samples
        self.processeddata = [] #will store all processed values
    
    def process_sample(self, val):
        val += -32678 #center on zero
        self.windowedrawdata.append(val)
        if len(self.windowedrawdata) > 500: #TODO: Could redefine process_sample function during runtime once the buffer is built up
            self.windowedrawdata.pop(0)
        hpFilteredData= signal.lfilter(EMGChannel.b,EMGChannel.a, self.windowedrawdata)
        recData= abs(hpFilteredData)
        filteredData = signal.lfilter(EMGChannel.bl, EMGChannel.al, recData)
        return filteredData[-1], hpFilteredData[-1], recData[-1]
        
    def process_sample_log(self, val):
        self.rawdata.append(val)
        filteredVal = process_sample(val)
        self.processeddata.append(filteredVal)
        return filteredVal
        
    def get_raw_data(self):
        return self.rawdata
        
    def get_processed_data(self):
        return self.processeddata
