FROM alpine:latest
RUN apk add --no-cache py-pip bash
RUN pip install codespell
WORKDIR /home
COPY ignore.txt ignore.txt
