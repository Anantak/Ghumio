reset
set key maxrows 3
plot "TargetKinFit.Pre2.data.csv" u 20:1 with linespoints lt 7 pt 28 title "Pre2x", \
     "TargetKinFit.Pre2.data.csv" u 20:2 with linespoints lt 7 pt 28 title "Pre2y", \
     "TargetKinFit.Pre2.data.csv" u 20:3 with linespoints lt 7 pt 28 title "Pre2z", \
     "TargetKinFit.1.data.csv" u 20:1 with linespoints lt 1 pt 1 title "1x", \
     "TargetKinFit.1.data.csv" u 20:2 with linespoints lt 1 pt 2 title "1y", \
     "TargetKinFit.1.data.csv" u 20:3 with linespoints lt 1 pt 3 title "1z", \
     "TargetKinFit.2.data.csv" u 20:1 with linespoints lt 2 pt 1 title "2x", \
     "TargetKinFit.2.data.csv" u 20:2 with linespoints lt 2 pt 2 title "2y", \
     "TargetKinFit.2.data.csv" u 20:3 with linespoints lt 2 pt 3 title "2z", \
     "TargetPosesInterp.2.data.csv"  u  8:1 with linespoints lt 9 pt 29 title "Txi", \
     "TargetPosesInterp.2.data.csv"  u  8:2 with linespoints lt 9 pt 29 title "Tyi", \
     "TargetPosesInterp.2.data.csv"  u  8:3 with linespoints lt 9 pt 29 title "Tzi", \
     "TargetPoses.2.data.csv"  u  8:1 with linespoints lt 8 pt 30 title "Tx", \
     "TargetPoses.2.data.csv"  u  8:2 with linespoints lt 8 pt 30 title "Ty", \
     "TargetPoses.2.data.csv"  u  8:3 with linespoints lt 8 pt 30 title "Tz"

