Generate new report:

```
sudo nsys profile --trace=cuda,nvtx,osrt --output=/home/nicolas/dev/research/KGMT/NSight/review --force-overwrite=true /home/nicolas/dev/research/KGMT/build/benchMain
```

Look at new report:
nsys stats --force-export=true /home/nicolas/dev/research/KGMT/NSight/review.nsys-rep | grep -v "SKIPPED"

Look at Baseline:

```
nsys stats /home/nicolas/dev/research/KGMT/NSight/baseline.nsys-rep | grep -v "SKIPPED"
```
