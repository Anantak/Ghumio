reset
set view equal xyz
plot "TargetKinFit.1.data.csv" u 20:5 with linespoints lt 1 pt 1, \
     "TargetKinFit.1.data.csv" u 20:6 with linespoints lt 1 pt 2, \
     "TargetKinFit.1.data.csv" u 20:7 with linespoints lt 1 pt 3, \
     "TargetKinFit.2.data.csv" u 20:5 with linespoints lt 2 pt 1, \
     "TargetKinFit.2.data.csv" u 20:6 with linespoints lt 2 pt 2, \
     "TargetKinFit.2.data.csv" u 20:7 with linespoints lt 2 pt 3, \
     "TargetKinFit.3.data.csv" u 20:5 with linespoints lt 3 pt 1, \
     "TargetKinFit.3.data.csv" u 20:6 with linespoints lt 3 pt 2, \
     "TargetKinFit.3.data.csv" u 20:7 with linespoints lt 3 pt 3, \
     "TargetKinFit.4.data.csv" u 20:5 with linespoints lt 4 pt 1, \
     "TargetKinFit.4.data.csv" u 20:6 with linespoints lt 4 pt 2, \
     "TargetKinFit.4.data.csv" u 20:7 with linespoints lt 4 pt 3, \
     "TargetKinFit.5.data.csv" u 20:5 with linespoints lt 5 pt 1, \
     "TargetKinFit.5.data.csv" u 20:6 with linespoints lt 5 pt 2, \
     "TargetKinFit.5.data.csv" u 20:7 with linespoints lt 5 pt 3
