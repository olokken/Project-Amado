## Prerequisites 
- Docker installed and running 
- VsCode Devcontainer extension
- NoVNC container running
- A Docker network called Ros


## noVNC
- Run the following commands to start the noVNC
```bash 
docker pull theasp/novnc:latest

docker run -d --rm --net=ros \
   --env="DISPLAY_WIDTH=3000" \
   --env="DISPLAY_HEIGHT=1800" \
   --env="RUN_XTERM=no" \
   --name=novnc -p=8080:8080 \
   theasp/novnc:latest  
   ```

- Find GUI at http://localhost:8080/vnc.html