velocity-obstacle basic

```
ffmpeg -t 5 -pattern_type glob -i '*.png' -vf "scale=1280:-1:flags=lanczos,split[s0][s1];[s0]palettegen[p];[s1][p]paletteuse" -loop 0 output.gif

ffmpeg -i 2-5-slowpoke.gif -movflags faststart -pix_fmt yuv420p -vf "scale=trunc(iw/2)*2:trunc(ih/2)*2" out.mp4
```

## 1-sitting-duck
![scenario1](https://github.com/nilay994/trytime/raw/master/velobs/1-sitting-duck.gif "scenario1")
 
## 2-slowpoke-intruder
![scenario2](https://github.com/nilay994/trytime/raw/master/velobs/2-slowpoke-intruder.gif "scenario2")

## 2-5-slowpoke
![scenario2-5](https://github.com/nilay994/trytime/raw/master/velobs/2-5-slowpoke.gif "scenario2-5")

## 3-oncoming-wrong-lane-dude
![scenario3](https://github.com/nilay994/trytime/raw/master/velobs/3-oncoming-wrong-lane-dude.gif "scenario3")

## 4-dont-cross-me
![scenario4](https://github.com/nilay994/trytime/raw/master/velobs/4-dont-cross-me.gif "scenario4")

## 5-excuse-me
![scenario5](https://github.com/nilay994/trytime/raw/master/velobs/5-excuse-me.gif "scenario5")
