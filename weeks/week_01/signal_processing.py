import numpy as np
import pandas as pd
from numpy import fft


def compute_spectrum(series: pd.Series, fs: float) -> tuple[np.ndarray, np.ndarray]:
    """
    Computes the fft of *series* at a given sample rate *fs* and returns the
    calculated frequencies and magnitudes

    Parameters
    ----------
    series: Series
        data series being computed
    fs: float
        sampling frequency
    Returns
    -------
    freq, mag: ndarray, ndarray
        FFT frequencies *freq* and associated magnitudes *mag* computed for the
        given data series

    """
    freq = fft.rfftfreq(len(series), d=1 / fs)
    mag = abs(fft.rfft(series.values))
    return freq, mag
