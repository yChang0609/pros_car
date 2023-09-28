export PROJECT_NAME="new_project"
export PROJECT_URL="new_project_url"
git clone ssh://git@paia-tech.synology.me:8222/pros/pros-boilerplate.git $PROJECT_NAME
cd $PROJECT_NAME
git remote update origin $PROJECT_URL
git branch -M main
git push -uf origin main

