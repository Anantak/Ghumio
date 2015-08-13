reset
set key maxrows 4
plot "TargetKinFit.Pre3.data.csv" u 20:1 with linespoints lt 2 pt 1 title "Pre3x", \
     "TargetKinFit.Pre3.data.csv" u 20:2 with linespoints lt 2 pt 4 title "Pre3y", \
     "TargetKinFit.Pre3.data.csv" u 20:3 with linespoints lt 2 pt 6 title "Pre3z", \
     "TargetKinFit.Pre3.data.csv" u 20:4 with linespoints lt 2 pt 8 title "Pre3w", \
     "TargetKinFit.1.data.csv" u 20:1 with linespoints lt 1 pt 1 title "1x", \
     "TargetKinFit.1.data.csv" u 20:2 with linespoints lt 1 pt 2 title "1y", \
     "TargetKinFit.1.data.csv" u 20:3 with linespoints lt 1 pt 3 title "1z", \
     "TargetKinFit.1.data.csv" u 20:4 with linespoints lt 1 pt 4 title "1w", \
     "TargetKinFit.2.data.csv" u 20:1 with linespoints lt 2 pt 1 title "2x", \
     "TargetKinFit.2.data.csv" u 20:2 with linespoints lt 2 pt 2 title "2y", \
     "TargetKinFit.2.data.csv" u 20:3 with linespoints lt 2 pt 3 title "2z", \
     "TargetKinFit.2.data.csv" u 20:4 with linespoints lt 2 pt 4 title "2w", \
     "TargetKinFit.3.data.csv" u 20:1 with linespoints lt 3 pt 1 title "3x", \
     "TargetKinFit.3.data.csv" u 20:2 with linespoints lt 3 pt 2 title "3y", \
     "TargetKinFit.3.data.csv" u 20:3 with linespoints lt 3 pt 3 title "3z", \
     "TargetKinFit.3.data.csv" u 20:4 with linespoints lt 3 pt 4 title "3w", \
     "TargetPosesInterp.3.data.csv"  u  8:1 with linespoints lt 9 pt 29 title "Txi", \
     "TargetPosesInterp.3.data.csv"  u  8:2 with linespoints lt 9 pt 29 title "Tyi", \
     "TargetPosesInterp.3.data.csv"  u  8:3 with linespoints lt 9 pt 29 title "Tzi", \
     "TargetPosesInterp.3.data.csv"  u  8:4 with linespoints lt 9 pt 0 title "Twi", \
     "TargetPoses.3.data.csv"  u  8:1 with linespoints lt 8 pt 30 title "Tx", \
     "TargetPoses.3.data.csv"  u  8:2 with linespoints lt 8 pt 30 title "Ty", \
     "TargetPoses.3.data.csv"  u  8:3 with linespoints lt 8 pt 30 title "Tz", \
     "TargetPoses.3.data.csv"  u  8:4 with linespoints lt 8 pt 0 title "Tw"

