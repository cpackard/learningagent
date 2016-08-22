ffmpeg -i all_polys%07d.png -c:v libx264 -r 30 -pix_fmt yuv420p -vf "scale=trunc(iw/2)*2:trunc(ih/2)*2"  agent_run.mp4
