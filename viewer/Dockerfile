FROM node:8.16.0-alpine

COPY . /openvslam-viewer/

RUN set -x && \
  cd /openvslam-viewer/ && \
  npm install

ENTRYPOINT ["node", "/openvslam-viewer/app.js"]
