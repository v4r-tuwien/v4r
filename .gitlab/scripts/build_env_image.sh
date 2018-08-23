#!/bin/sh

if [ -z ${CI_REGISTRY_IMAGE+x} ]; then
  echo "Container registry is disabled for this project"
  echo "Navigate to Settings > Permissions > Repository and switch on \"Container registry\""
  exit 1
fi

IMAGE_NAME=${CI_REGISTRY_IMAGE}/env/${UBUNTU_DISTRO}
IMAGE_PIPELINE_TAG=${CI_REGISTRY_IMAGE}/env/${UBUNTU_DISTRO}:${CI_PIPELINE_ID}
IMAGE_LATEST_TAG=${CI_ORIGIN_REGISTRY_IMAGE}/env/${UBUNTU_DISTRO}:latest

for f in setup.sh package.xml; do
  cp $f .gitlab/docker/env
done

docker pull ${IMAGE_LATEST_TAG}

docker build \
  -t ${IMAGE_PIPELINE_TAG} \
  --cache-from ${IMAGE_LATEST_TAG} \
  --build-arg UBUNTU_DISTRO=${UBUNTU_DISTRO} \
  --build-arg ROS_DISTRO=${ROS_DISTRO} \
  .gitlab/docker/env

if [ "${CI_COMMIT_REF_NAME}" == "master" ] && [ "${CI_PROJECT_NAMESPACE}" == "${CI_ORIGIN_NAMESPACE}" ]; then
  docker tag ${IMAGE_PIPELINE_TAG} ${IMAGE_LATEST_TAG}
  docker push ${IMAGE_LATEST_TAG}
fi

docker push ${IMAGE_PIPELINE_TAG}
docker rmi ${IMAGE_PIPELINE_TAG}
