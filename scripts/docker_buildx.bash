# !!! should be at project root folder

# login aws 
aws ecr-public get-login-password --region us-east-1 | docker login --username AWS --password-stdin public.ecr.aws/paia-tech
# set AWS 
export IMG_NAME="pros-car"
export ECR_URL="public.ecr.aws/paia-tech"
export TODAY=$(date +%Y%m%d)

docker buildx build --platform linux/amd64,linux/arm64 \
    -t $IMG_NAME:$TODAY \
    -t $ECR_URL/$IMG_NAME:latest \
    -t $ECR_URL/$IMG_NAME:$TODAY \
    . --push
