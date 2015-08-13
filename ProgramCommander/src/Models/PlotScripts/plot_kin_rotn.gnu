reset
set key maxrows 6 font ",6"
plot "TargetKinFit.Pre0.data.csv" u 20:1 with linespoints lt 7 pt 28 title "Pre0x", \
     "TargetKinFit.Pre0.data.csv" u 20:2 with linespoints lt 7 pt 28 title "Pre0y", \
     "TargetKinFit.Pre0.data.csv" u 20:3 with linespoints lt 7 pt 28 title "Pre0z", \
     "TargetKinFit.Pre9.data.csv" u 20:1 with linespoints lt 6 pt 28 title "Pre9x", \
     "TargetKinFit.Pre9.data.csv" u 20:2 with linespoints lt 6 pt 28 title "Pre9y", \
     "TargetKinFit.Pre9.data.csv" u 20:3 with linespoints lt 6 pt 28 title "Pre9z", \
     "TargetKinFit.Pre8.data.csv" u 20:1 with linespoints lt 5 pt 28 title "Pre8x", \
     "TargetKinFit.Pre8.data.csv" u 20:2 with linespoints lt 5 pt 28 title "Pre8y", \
     "TargetKinFit.Pre8.data.csv" u 20:3 with linespoints lt 5 pt 28 title "Pre8z", \
     "TargetPoses.0.data.csv"  u  8:1 with linespoints lt 8 pt 6 ps 1.5 title "Tx", \
     "TargetPoses.0.data.csv"  u  8:2 with linespoints lt 8 pt 6 ps 1.5 title "Ty", \
     "TargetPoses.0.data.csv"  u  8:3 with linespoints lt 8 pt 6 ps 1.5 title "Tz", \
     "TargetPosesInterp.0.data.csv"  u  8:1 with linespoints lt 7 pt 9 title "Txi", \
     "TargetPosesInterp.0.data.csv"  u  8:2 with linespoints lt 7 pt 9 title "Tyi", \
     "TargetPosesInterp.0.data.csv"  u  8:3 with linespoints lt 7 pt 9 title "Tzi", \
     "TargetKinFit.1.data.csv" u 20:1 with linespoints lt 1 pt 1 title "1x", \
     "TargetKinFit.1.data.csv" u 20:2 with linespoints lt 1 pt 2 title "1y", \
     "TargetKinFit.1.data.csv" u 20:3 with linespoints lt 1 pt 3 title "1z", \
     "TargetKinFit.2.data.csv" u 20:1 with linespoints lt 2 pt 1 title "2x", \
     "TargetKinFit.2.data.csv" u 20:2 with linespoints lt 2 pt 2 title "2y", \
     "TargetKinFit.2.data.csv" u 20:3 with linespoints lt 2 pt 3 title "2z", \
     "TargetKinFit.3.data.csv" u 20:1 with linespoints lt 3 pt 1 title "3x", \
     "TargetKinFit.3.data.csv" u 20:2 with linespoints lt 3 pt 2 title "3y", \
     "TargetKinFit.3.data.csv" u 20:3 with linespoints lt 3 pt 3 title "3z", \
     "TargetKinFit.4.data.csv" u 20:1 with linespoints lt 4 pt 1 title "4x", \
     "TargetKinFit.4.data.csv" u 20:2 with linespoints lt 4 pt 2 title "4y", \
     "TargetKinFit.4.data.csv" u 20:3 with linespoints lt 4 pt 3 title "4z", \
     "TargetKinFit.5.data.csv" u 20:1 with linespoints lt 5 pt 1 title "5x", \
     "TargetKinFit.5.data.csv" u 20:2 with linespoints lt 5 pt 2 title "5y", \
     "TargetKinFit.5.data.csv" u 20:3 with linespoints lt 5 pt 3 title "5z", \
     "TargetKinFit.6.data.csv" u 20:1 with linespoints lt 6 pt 1 title "6x", \
     "TargetKinFit.6.data.csv" u 20:2 with linespoints lt 6 pt 2 title "6y", \
     "TargetKinFit.6.data.csv" u 20:3 with linespoints lt 6 pt 3 title "6z", \
     "TargetKinFit.7.data.csv" u 20:1 with linespoints lt 1 pt 1 title "7x", \
     "TargetKinFit.7.data.csv" u 20:2 with linespoints lt 1 pt 2 title "7y", \
     "TargetKinFit.7.data.csv" u 20:3 with linespoints lt 1 pt 3 title "7z", \
     "TargetKinFit.8.data.csv" u 20:1 with linespoints lt 2 pt 1 title "8x", \
     "TargetKinFit.8.data.csv" u 20:2 with linespoints lt 2 pt 2 title "8y", \
     "TargetKinFit.8.data.csv" u 20:3 with linespoints lt 2 pt 3 title "8z", \
     "TargetKinFit.9.data.csv" u 20:1 with linespoints lt 3 pt 1 title "9x", \
     "TargetKinFit.9.data.csv" u 20:2 with linespoints lt 3 pt 2 title "9y", \
     "TargetKinFit.9.data.csv" u 20:3 with linespoints lt 3 pt 3 title "9z", \
     "TargetKinFit.0.data.csv" u 20:1 with linespoints lt 4 pt 1 title "0x", \
     "TargetKinFit.0.data.csv" u 20:2 with linespoints lt 4 pt 2 title "0y", \
     "TargetKinFit.0.data.csv" u 20:3 with linespoints lt 4 pt 3 title "0z"

