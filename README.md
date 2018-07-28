# fwrepo
mongo db installation step:
1,check mongodb in docker
  sudo docker search  mongo
2,download 
docker pull mongo:3.2
3,run mongo
sudo docker run -p 27018:27017 -v $PWD/db:/data/db -d mongo:3.2
4,login mongodb 
docker exec -it #containerid# bash
mongo
