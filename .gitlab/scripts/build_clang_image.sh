#!/bin/sh

if [ -z ${CI_REGISTRY_IMAGE+x} ]; then
  echo "Container registry is disabled for this project"
  echo "Navigate to Settings > Permissions > Repository and switch on \"Container registry\""
  exit 1
fi

IMAGE_NAME=${CI_ORIGIN_REGISTRY_IMAGE}/clang

docker pull ${IMAGE_NAME}

if [ $? -ne 0 ] && [ "${CI_PROJECT_NAMESPACE}" == "${CI_ORIGIN_NAMESPACE}" ]; then
  docker build -t ${IMAGE_NAME} .gitlab/docker/clang
  docker push ${IMAGE_NAME}
fi
