import pandas as pd
import numpy as np


df = pd.read_excel('data_to_interpolate_scenario2_ensemble.xlsx')
df['x'].replace(0, np.nan, inplace=True)
df['y'].replace(0, np.nan, inplace=True)
df['x1'].replace(0, np.nan, inplace=True)
df['y1'].replace(0, np.nan, inplace=True)

df['x'] = df['x'].interpolate(method='linear')
df['y'] = df['y'].interpolate(method='linear')
df['x1'] = df['x1'].interpolate(method='linear')
df['y1'] = df['y1'].interpolate(method='linear')

df['x'].fillna(method='ffill', inplace=True)  # Forward fill
df['y'].fillna(method='ffill', inplace=True)  # Forward fill
df['x1'].fillna(method='ffill', inplace=True)  # Forward fill
df['y1'].fillna(method='ffill', inplace=True)  # Forward fill

df.to_excel('Interpolated_data_scenario2_ensemble.xlsx', index=False)
