#!/usr/bin/env python3

import numpy as np
import math
import os
import matplotlib.pyplot as plt
from numpy.core.fromnumeric import size
import pandas as pd


w = 0.6
font = 24
############################################################################################################
# INTERFACE HELPFULESS AND EASE OF USE
############################################################################################################

# df1 = pd.read_csv(r'/home/kenembanisi/Documents/data/helpfulness.csv')
# df2 = pd.read_csv(r'/home/kenembanisi/Documents/data/ease_of_use.csv')

# conditions = ['MC', 'H', 'V-T', 'V-B', 'HV-T', 'HV-B']
# #---
# y1 = [np.mean(df1[c]) for c in conditions]
# yerror1 = [np.std(df1[c]) / np.sqrt(np.size(df1[c])) for c in conditions]
# #---
# y2 = [np.mean(df2[c]) for c in conditions]
# yerror2 = [np.std(df2[c]) / np.sqrt(np.size(df2[c])) for c in conditions]
# #---
# x = np.arange(len(conditions)) # the label locations
# width = 0.4 # the width of the bars

# fig, ax = plt.subplots()
# r1 = ax.bar(x - width/2, y1, yerr=yerror1, align='center', alpha=0.4, ecolor='black', capsize=8, width = width, label='Helpfulness')
# r2 = ax.bar(x + width/2, y2, yerr=yerror2, align='center', alpha=0.4, color ='r', ecolor='black', capsize=8, width = width, label='Ease of Use')

# # ax.set_ylabel('Likert Rating', fontsize=font)
# ax.set_xticks(x)
# ax.set_xticklabels(conditions)
# ax.tick_params(axis='both', labelsize=font)
# ax.set_title('Helpfuless and Ease of Use', fontsize=22)
# ax.legend(fontsize=17, ncol=2)

# ax.set_ylim(0,7.5)

# # Save the figure and show
# plt.tight_layout()
# plt.show()

# plt.savefig("helpfulness_ease_of_use.pdf", dpi=1000)


############################################################################################################
# INTENT UNDERSTANDING
############################################################################################################

# df1 = pd.read_csv(r'/home/kenembanisi/Documents/data/intent_understanding.csv')

# conditions = ['H', 'V-T', 'V-B', 'HV-T', 'HV-B']
# #---
# y1 = [np.mean(df1[c]) for c in conditions]
# yerror1 = [np.std(df1[c]) / np.sqrt(np.size(df1[c])) for c in conditions]
# #---
# x = np.arange(len(conditions)) # the label locations
# # w = 0.4 # the width of the bars

# fig, ax = plt.subplots()
# r1 = ax.bar(x, y1, yerr=yerror1, align='center', alpha=0.4, ecolor='black', capsize=10, width = w)

# # ax.set_ylabel('Likert Rating', fontsize=15)
# ax.set_xticks(x)
# ax.set_xticklabels(conditions)
# ax.tick_params(axis='both', labelsize=font)
# ax.set_title('Intent Understanding', fontsize=font)
# # ax.legend(fontsize=16, ncol=2)

# ax.set_ylim(0,7.5)

# # Save the figure and show
# plt.tight_layout()
# # plt.show()

# plt.savefig("intent_understanding.pdf", dpi=1000)



############################################################################################################
# FORCE ANTICIPATION
############################################################################################################

# df1 = pd.read_csv(r'/home/kenembanisi/Documents/data/anticipation.csv')

# conditions = ['H', 'HV-T', 'HV-B']
# #---
# y1 = [np.mean(df1[c]) for c in conditions]
# yerror1 = [np.std(df1[c]) / np.sqrt(np.size(df1[c])) for c in conditions]
# #---
# x = np.arange(len(conditions)) # the label locations
# # w = 0.4 # the width of the bars

# fig, ax = plt.subplots()
# r1 = ax.bar(x, y1, yerr=yerror1, align='center', alpha=0.4, ecolor='black', capsize=10, width = w)

# # ax.set_ylabel('Likert Rating', fontsize=15)
# ax.set_xticks(x)
# ax.set_xticklabels(conditions)
# ax.tick_params(axis='both', labelsize=font)
# ax.set_title('Force Anticipation', fontsize=font)
# # ax.legend(fontsize=16, ncol=2)

# ax.set_ylim(0,7.5)

# # Save the figure and show
# plt.tight_layout()
# # plt.show()

# plt.savefig("anticipation.pdf", dpi=1000)


############################################################################################################
# COOPERATION
############################################################################################################

# df1 = pd.read_csv(r'/home/kenembanisi/Documents/data/cooperation.csv')

# conditions = ['H', 'V-T', 'V-B', 'HV-T', 'HV-B']
# #---
# y1 = [np.mean(df1[c]) for c in conditions]
# yerror1 = [np.std(df1[c]) / np.sqrt(np.size(df1[c])) for c in conditions]
# #---
# x = np.arange(len(conditions)) # the label locations
# # w = 0.4 # the width of the bars

# fig, ax = plt.subplots()
# r1 = ax.bar(x, y1, yerr=yerror1, align='center', alpha=0.4, ecolor='black', capsize=10, width = w)

# # ax.set_ylabel('Likert Rating', fontsize=15)
# ax.set_xticks(x)
# ax.set_xticklabels(conditions)
# ax.tick_params(axis='both', labelsize=font)
# ax.set_title('Cooperation', fontsize=font)
# # ax.legend(fontsize=16, ncol=2)

# ax.set_ylim(0,7.5)

# # Save the figure and show
# plt.tight_layout()
# # plt.show()

# plt.savefig("cooperation.pdf", dpi=1000)


# ############################################################################################################
# # SENSE OF CONTROL
# ############################################################################################################

# df1 = pd.read_csv(r'/home/kenembanisi/Documents/data/control.csv')

# conditions = ['MC', 'H', 'V-T', 'V-B', 'HV-T', 'HV-B']
# #---
# y1 = [np.mean(df1[c]) for c in conditions]
# yerror1 = [np.std(df1[c]) / np.sqrt(np.size(df1[c])) for c in conditions]
# #---
# x = np.arange(len(conditions)) # the label locations
# # w = 0.4 # the width of the bars

# fig, ax = plt.subplots()
# r1 = ax.bar(x, y1, yerr=yerror1, align='center', alpha=0.4, ecolor='black', capsize=10, width = w)

# # ax.set_ylabel('Likert Rating', fontsize=15)
# ax.set_xticks(x)
# ax.set_xticklabels(conditions)
# ax.tick_params(axis='both', labelsize=font)
# ax.set_title('Sense of Control', fontsize=font)
# # ax.legend(fontsize=16, ncol=2)

# ax.set_ylim(0,7.5)

# # Save the figure and show
# plt.tight_layout()
# # plt.show()

# plt.savefig("sense_of_control.pdf", dpi=1000)



############################################################################################################
# SITUATION AWARENESS
############################################################################################################

# df1 = pd.read_csv(r'/home/kenembanisi/Documents/data/info_gain.csv')
# df2 = pd.read_csv(r'/home/kenembanisi/Documents/data/info_usefulness.csv')

# conditions = ['MC', 'H', 'V-T', 'V-B', 'HV-T', 'HV-B']
# #---
# y1 = [np.mean(df1[c]) for c in conditions]
# yerror1 = [np.std(df1[c]) / np.sqrt(np.size(df1[c])) for c in conditions]
# #---
# y2 = [np.mean(df2[c]) for c in conditions]
# yerror2 = [np.std(df2[c]) / np.sqrt(np.size(df2[c])) for c in conditions]
# #---
# x = np.arange(len(conditions)) # the label locations
# w = 0.4 # the width of the bars

# fig, ax = plt.subplots()
# r1 = ax.bar(x - w/2, y1, yerr=yerror1, align='center', alpha=0.4, ecolor='black', capsize=8, width = w, label='Information Gain')
# r2 = ax.bar(x + w/2, y2, yerr=yerror2, align='center', alpha=0.4, color ='r', ecolor='black', capsize=8, width = w, label='Information Usefulness')

# # ax.set_ylabel('Situation Awareness', fontsize=16)
# ax.set_title('Situation Awareness', fontsize=font)
# ax.set_xticks(x)
# ax.set_xticklabels(conditions)
# ax.tick_params(axis='both', labelsize=font)
# ax.legend(fontsize=16, ncol=2)

# ax.set_ylim(0,7.5)

# # Save the figure and show
# plt.tight_layout()
# plt.show()

# plt.savefig("situation_awareness.pdf", dpi=1000)



############################################################################################################
# NO. OF INTRUSIONS
############################################################################################################

# df1 = pd.read_csv(r'/home/kenembanisi/Documents/data/intrusions.csv')

# conditions = ['MC', 'H', 'V-T', 'V-B', 'HV-T', 'HV-B']
# #---
# y1 = [np.mean(df1[c]) for c in conditions]
# yerror1 = [np.std(df1[c]) / np.sqrt(np.size(df1[c])) for c in conditions]
# #---
# x = np.arange(len(conditions)) # the label locations
# w = 0.4 # the width of the bars

# fig, ax = plt.subplots()
# r1 = ax.bar(x, y1, yerr=yerror1, align='center', alpha=0.4, ecolor='black', capsize=5, width = w)

# # ax.set_ylabel('Likert Rating', fontsize=15)
# ax.set_xticks(x)
# ax.set_xticklabels(conditions)
# ax.tick_params(axis='both', labelsize=16)
# ax.set_title('Avg. Number of Intimate Intrusions (Per Trial)', fontsize=16)
# # ax.legend(fontsize=16, ncol=2)

# # ax.set_ylim(0,7.5)

# # Save the figure and show
# plt.tight_layout()
# # plt.show()

# plt.savefig("intrusions.pdf", dpi=1000)


############################################################################################################
# TIME TO INTRUSION
############################################################################################################

# df1 = pd.read_csv(r'/home/kenembanisi/Documents/data/time_to_intrusion.csv')

# conditions = ['MC', 'H', 'V-T', 'V-B', 'HV-T', 'HV-B']
# #---
# y1 = [np.mean(df1[c]) for c in conditions]
# yerror1 = [np.std(df1[c]) / np.sqrt(np.size(df1[c])) for c in conditions]
# #---
# x = np.arange(len(conditions)) # the label locations
# w = 0.5 # the width of the bars

# fig, ax = plt.subplots()
# r1 = ax.bar(x, y1, yerr=yerror1, align='center', alpha=0.4, ecolor='black', capsize=5, width = w)

# # ax.set_ylabel('Likert Rating', fontsize=15)
# ax.set_xticks(x)
# ax.set_xticklabels(conditions)
# ax.tick_params(axis='both', labelsize=16)
# ax.set_title('Avg. Minimum Time to Intrusion', fontsize=16)
# # ax.legend(fontsize=16, ncol=2)

# # ax.set_ylim(0,7.5)

# # Save the figure and show
# plt.tight_layout()
# # plt.show()

# plt.savefig("time_to_intrusions.pdf", dpi=1000)



# ############################################################################################################
# # MEAN DISAGREEMENT
# ############################################################################################################

# df1 = pd.read_csv(r'/home/kenembanisi/Documents/data/disagreement.csv')

# conditions = ['H', 'V-T', 'V-B', 'HV-T', 'HV-B']
# #---
# y1 = [np.mean(df1[c]) for c in conditions]
# yerror1 = [np.std(df1[c]) / np.sqrt(np.size(df1[c])) for c in conditions]
# #---
# x = np.arange(len(conditions)) # the label locations
# w = 0.5 # the width of the bars

# fig, ax = plt.subplots()
# r1 = ax.bar(x, y1, yerr=yerror1, align='center', alpha=0.4, ecolor='black', capsize=5, width = w)

# # ax.set_ylabel('Likert Rating', fontsize=15)
# ax.set_xticks(x)
# ax.set_xticklabels(conditions)
# ax.tick_params(axis='both', labelsize=16)
# ax.set_title('Mean Disagreement', fontsize=16)
# # ax.legend(fontsize=16, ncol=2)

# # ax.set_ylim(0,7.5)

# # Save the figure and show
# plt.tight_layout()
# # plt.show()

# plt.savefig("disagreement.pdf", dpi=1000)


############################################################################################################
# INTENT UNDERSTANDING - GAMING EXPERIENCE
############################################################################################################

# df1 = pd.read_csv(r'/home/kenembanisi/Documents/data/intent_understanding_gaming.csv')

# conditions = ['H', 'V-T', 'V-B', 'HV-T', 'HV-B']
# #---
# y1 = [np.mean(df1[c][9:]) for c in conditions]
# yerror1 = [np.std(df1[c][9:]) / np.sqrt(np.size(df1[c][9:])) for c in conditions]
# #---
# y2 = [np.mean(df1[c][0:8]) for c in conditions]
# yerror2 = [np.std(df1[c][0:8]) / np.sqrt(np.size(df1[c][0:8])) for c in conditions]
# #---
# x = np.arange(len(conditions)) # the label locations
# w = 0.4 # the width of the bars

# fig, ax = plt.subplots()
# r1 = ax.bar(x - w/2, y1, yerr=yerror1, align='center', alpha=0.4, ecolor='black', capsize=8, width = w, label='Non-Gamer')
# r2 = ax.bar(x + w/2, y2, yerr=yerror2, align='center', alpha=0.4, color ='r', ecolor='black', capsize=8, width = w, label='Gamer')

# # ax.set_ylabel('Situation Awareness', fontsize=16)
# ax.set_title('Intent Understanding', fontsize=font)
# ax.set_xticks(x)
# ax.set_xticklabels(conditions)
# ax.tick_params(axis='both', labelsize=font)
# ax.legend(fontsize=18, ncol=2)

# ax.set_ylim(0,8)

# # Save the figure and show
# plt.tight_layout()
# # plt.show()

# plt.savefig("intent_understanding_gaming.pdf", dpi=1000)


############################################################################################################
# INTENT UNDERSTANDING - GAMING EXPERIENCE
############################################################################################################

# df1 = pd.read_csv(r'/home/kenembanisi/Documents/data/anticipation_gaming.csv')

# conditions = ['H', 'HV-T', 'HV-B']
# #---
# y1 = [np.mean(df1[c][9:]) for c in conditions]
# yerror1 = [np.std(df1[c][9:]) / np.sqrt(np.size(df1[c][9:])) for c in conditions]
# #---
# y2 = [np.mean(df1[c][0:8]) for c in conditions]
# yerror2 = [np.std(df1[c][0:8]) / np.sqrt(np.size(df1[c][0:8])) for c in conditions]
# #---
# x = np.arange(len(conditions)) # the label locations
# w = 0.4 # the width of the bars

# fig, ax = plt.subplots()
# r1 = ax.bar(x - w/2, y1, yerr=yerror1, align='center', alpha=0.4, ecolor='black', capsize=8, width = w, label='Non-Gamer')
# r2 = ax.bar(x + w/2, y2, yerr=yerror2, align='center', alpha=0.4, color ='r', ecolor='black', capsize=8, width = w, label='Gamer')

# # ax.set_ylabel('Situation Awareness', fontsize=16)
# ax.set_title('Force Anticipation', fontsize=font)
# ax.set_xticks(x)
# ax.set_xticklabels(conditions)
# ax.tick_params(axis='both', labelsize=font)
# ax.legend(fontsize=18, ncol=2)

# ax.set_ylim(0,8)

# # Save the figure and show
# plt.tight_layout()
# # plt.show()

# plt.savefig("force_anticipation_gaming.pdf", dpi=1000)



############################################################################################################
# PREFERENCE RANKING
############################################################################################################

df1 = pd.read_csv(r'/home/kenembanisi/Documents/data/preference.csv')

conditions = ['MC', 'H', 'V-T', 'V-B', 'HV-T', 'HV-B']
positions = ['1st', '2nd', '3rd', '4th', '5th', '6th']

df_t = df1.T # transpose the dataframe
# totals = 15

# MC = [ df_t[i][1]/15 for i in range(0,6)]
# H = [ df_t[i][2]/15 for i in range(0,6)]
# VT = [ df_t[i][3]/15 for i in range(0,6)]

# x = np.arange(len(positions)) # the label locations
# w = 0.5 # the width of the bars

# fig, ax = plt.subplots()
# plt.bar(x, MC, color='#b5ffb9', edgecolor='white', width=w)
# plt.bar(x, H, color='#f9bc86', edgecolor='white', width=w)
# plt.bar(x, VT, color='b', edgecolor='white', width=w)

# ax.set_xticks(x)
# ax.set_xticklabels(positions)
print(df1)

df1.plot(
    x = 'rank',
    kind = 'bar',
    stacked = True,
    title = 'Stacked Bar Graph',
    mark_right = False)


plt.show()


