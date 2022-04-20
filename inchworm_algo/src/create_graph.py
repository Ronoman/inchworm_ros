#!/usr/bin/env python3

import csv, sys, pandas as pd
import matplotlib.pyplot as plt
import plotly.graph_objects as go

move_to_days = (16/3600)/24
WIDTH = 10
HEIGHT = 10
pattern = 0

data = []
with open(f"../data/{sys.argv[1]}") as csvfile:
    reader = csv.reader(csvfile, delimiter=' ')
    for row in reader:
        # print(row)
        row = row[0].split(',')
        row = [int(x) for x in row]
        row_data = {}
        row_data["inchworm_count"] = row[0]
        row_data["ticks"] = row[1]
        row_data["move_counts"] = row[2: 2+ row_data["inchworm_count"]]
        row_data["state_data"] = {
                "inchworm_id": [],
                "Pickup From Depot": [],
                "Move to Target": [], 
                "Install Shingle": [],
                "Move Shingle": [],
                "Explore": [],
                "Move to Depot": []
            }
        for i in range(row_data["inchworm_count"]):
            data_index = 2 + row_data["inchworm_count"] + i * 6
            row_sum = 0
            for i in range(6):
                row_sum += row[data_index + i]
            row_data["state_data"]['inchworm_id'].append(i)
            row_data["state_data"]['Pickup From Depot'].append(100 * row[data_index]/row_sum)
            row_data["state_data"]['Move to Target'].append(100 * row[data_index + 1]/row_sum)
            row_data["state_data"]['Install Shingle'].append(100 * row[data_index + 2]/row_sum)
            row_data["state_data"]['Move Shingle'].append(100 * row[data_index + 3]/row_sum)
            row_data["state_data"]['Explore'].append(100 * row[data_index + 4]/row_sum)
            row_data["state_data"]['Move to Depot'].append(100 * row[data_index + 5]/row_sum)

            # print(row_data["state_data"])
        data.append(row_data)
inchworms = []
move_data = []
average_data = pd.DataFrame()
for run in data:
    # print(run['state_data'])
    data_frame = pd.DataFrame.from_dict(run['state_data'], orient='index')
    print(data_frame.drop(['inchworm_id']).mean(axis=1).to_frame())
    print(average_data)
    run_average = data_frame.drop(['inchworm_id']).mean(axis=1)
    run_average.name = str(run["inchworm_count"])
    average_data = average_data.append(run_average)
    move_data.append(max(run['move_counts']) * move_to_days)
    inchworms.append(run['inchworm_count'])
    # print(run['move_counts'])
    fig = go.Figure()
    for i in range(run['inchworm_count']):
        print(data_frame.T.columns.values.tolist()[1:])
        print(data_frame.T.iloc[i].values.tolist()[1:])
        r_list = data_frame.T.iloc[i].values.tolist()[1:]
        r_list.append(data_frame.T.iloc[i].values.tolist()[1])
        theta_list = data_frame.T.columns.values.tolist()[1:]
        theta_list.append(data_frame.T.columns.values.tolist()[1])
        print(r_list)
        print(theta_list)
        fig.add_trace(go.Scatterpolar(
            r=r_list,
            theta=theta_list,
            fill='none',
            name=f'inchworm {i}'
        ))
    fig.update_traces(mode='lines+markers')
    fig.update_layout(
        polar=dict(
            radialaxis=dict(
            visible=True,
            range=[0, data_frame.max() + 20]
            )),
        showlegend=True
        )
    # fig.show()

print(average_data)

plt.scatter(inchworms, move_data)
plt.xlabel("Inchworm count")
plt.ylabel("Total estimated days")
plt.title(f"Time to shingle a {WIDTH}x{HEIGHT} roof with pattern {pattern}")

plt.savefig(f"{WIDTH}x{HEIGHT}_{pattern}.png")



fig = go.Figure()
for i in range(average_data.shape[0]):
    print(average_data.columns.values.tolist()[1:])
    print(average_data.iloc[i].values.tolist()[1:])
    r_list = average_data.iloc[i].values.tolist()[1:]
    r_list.append(average_data.iloc[i].values.tolist()[1])
    theta_list = average_data.columns.values.tolist()[1:]
    theta_list.append(average_data.columns.values.tolist()[1])
    fig.add_trace(go.Scatterpolar(
        r=r_list,
        theta=theta_list,
        fill='none',
        name=f'Run with {i + 1} inchworms'
    ))
fig.update_traces(mode='lines+markers')
fig.update_layout(
    polar=dict(
        radialaxis=dict(
        visible=True,
        range=[0, data_frame.max()],
        # title = "Percent of Actions Taken"
        )),
    showlegend=True,
    title=f"Share of Actions Taken in Run by Average Inchworm<br><sup>On a {WIDTH}x{HEIGHT} roof with pattern {pattern}</sup>",
    
    font=dict(
        family="Courier New, monospace",
        size=18
    )
    )
fig.write_image("test_figure.png", width=1000, height=600)
# fig.show()
# plt.show()