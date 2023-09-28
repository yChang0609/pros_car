export PROJECT_NAME="new_project"
export PROJECT_URL="new_project_url"
git clone ssh://git@paia-tech.synology.me:8222/pros/pros-boilerplate.git $PROJECT_NAME
cd $PROJECT_NAME
git remote rename origin old-origin
git remote add origin $PROJECT_URL
git tag | xargs git tag -d
git push -u origin --all
