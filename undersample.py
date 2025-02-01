import pandas as pd

# open file and delete one third of the line starting from the second line
with open('cart_race_track_trajectory.csv', 'r') as f:
    dataframe = pd.read_csv(f)
    dataframe = dataframe.iloc[::5]
    #write the new dataframe to the file as csv
    dataframe.to_csv("trajectory_downsampled.csv", index=False)
    f.close()