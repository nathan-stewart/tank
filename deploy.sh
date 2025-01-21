#!/bin/bash
# filepath: deploy.sh

# Replace with your actual deployment commands
rsync -avz --delete --exclude='node_modules' --exclude='.git' . nps@tank:/home/nps/tank
