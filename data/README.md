# data

## `Mo-Sys.nb`

Some Mathematica code to extract and plot the points from the Mo-Sys backups.

## `mosys010125`

Calibration data taken from the Mo-Sys backup, current as of 1 Jan 2025 I suppose, but it's old.

A CSV, `mosys010125.csv`

```
"P",4.8264,-5.30093,10.323,0,37,0,1,2,3,4,18,19,20,21,22,45,46,47,48,49,50,51,52,53,73,74,75,81,87,104,105,106,107,108,109,113,114,124,125,126,127,128,0,0,0,948.711,-1,0,0
```

Where, at lest for sure, the {2,3,4} are the $xyz$ of the reflector on the MAGIC ceiling. I think they are in meters. The remaining numbers are an interesting collection that may or may not identify the nearest neighbors of this point. A little Mathematica exploration should help.

A PNG, `mosys010125.png`

Made from the above, maybe useful for some template matching code.
