reset
set key maxrows 3
plot "TargetKinFit.Pre1.data.csv" u 20:8 with linespoints lt 7 pt 28 title "Pre1x", \
     "TargetKinFit.Pre1.data.csv" u 20:9 with linespoints lt 7 pt 28 title "Pre1y", \
     "TargetKinFit.Pre1.data.csv" u 20:10 with linespoints lt 7 pt 28 title "Pre1z", \
     "TargetKinFit.1.data.csv" u 20:8 with linespoints lt 1 pt 1 title "1x", \
     "TargetKinFit.1.data.csv" u 20:9 with linespoints lt 1 pt 2 title "1y", \
     "TargetKinFit.1.data.csv" u 20:10 with linespoints lt 1 pt 3 title "1z"

