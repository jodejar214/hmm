#!/usr/bin/env python
import csv
with open("results.csv","a") as f:
	fieldnames = ['Subject No.', 'Condition', 'Collision?', "Total Number of Path Calculations", "Travel Time", 'Anxious-Relaxed','Calm-Agitated',"Quiescent-Surprised", "Age", "Gender"]
	writer = csv.DictWriter(f, fieldnames=fieldnames)
	fields = {}
	for field in fieldnames:
		fields[field] = field
	writer.writerow(fields)
	with open("results.txt","r") as r:
		text = r.read()
		lines = text.split("\n\n\n")
		for l in lines:
			info = l.split("\n")
			infoDict = {}
			infoDict[fieldnames[0]] = info[0].split(" ")[1].strip()
			infoDict[fieldnames[1]] = info[1].split(" = ")[1].strip()
			infoDict[fieldnames[2]] = info[2].split(" ")[1].strip()
			infoDict[fieldnames[3]] = info[3].split(": ")[1].strip()
			infoDict[fieldnames[4]] = info[4].split(" ")[2].strip()
			infoDict[fieldnames[5]] = info[5]
			infoDict[fieldnames[6]] = info[6]
			infoDict[fieldnames[7]] = info[7]
			infoDict[fieldnames[8]] = info[8]
			infoDict[fieldnames[9]] = info[9]
			writer.writerow(infoDict)
