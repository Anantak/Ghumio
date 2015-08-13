reset
set key maxrows 3
plot "TargetKinFit.Pre1.data.csv" u 20:1 with linespoints lt 2 pt 1 title "Pre1x", \
     "TargetKinFit.Pre1.data.csv" u 20:2 with linespoints lt 2 pt 4 title "Pre1y", \
     "TargetKinFit.Pre1.data.csv" u 20:3 with linespoints lt 2 pt 6 title "Pre1z", \
     "TargetKinFit.1.data.csv" u 20:1 with linespoints lt 1 pt 1 title "1x", \
     "TargetKinFit.1.data.csv" u 20:2 with linespoints lt 1 pt 2 title "1y", \
     "TargetKinFit.1.data.csv" u 20:3 with linespoints lt 1 pt 3 title "1z", \
     "TargetPoses.1.data.csv"  u  8:1 with linespoints lt 8 pt 1 title "Tx", \
     "TargetPoses.1.data.csv"  u  8:2 with linespoints lt 8 pt 2 title "Ty", \
     "TargetPoses.1.data.csv"  u  8:3 with linespoints lt 8 pt 3 title "Tz"

