# !!! should be at project root folder

# login aws 
aws ecr-public get-login-password --region us-east-1 | docker login --username AWS --password-stdin public.ecr.aws/paia-tech
# set AWS 
export IMG_NAME="pros-car"
export ECR_URL="public.ecr.aws/paia-tech"
export TODAY=$(date +%Y%m%d)
docker build -t $IMG_NAME:latest -t $IMG_NAME:$TODAY -f ./Dockerfile .
docker tag $IMG_NAME:latest $ECR_URL/$IMG_NAME:latest
docker tag $IMG_NAME:latest $ECR_URL/$IMG_NAME:$TODAY

docker run \
    -it --rm --net=host \
    --cap-add=SYS_PTRACE --privileged\
    $IMG_NAME

# push the image to ECR
# docker push $ECR_URL/$IMG_NAME:latest
# docker push $ECR_URL/$IMG_NAME:$TODAY 