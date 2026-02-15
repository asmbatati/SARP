#!/bin/bash

# SAFEMRS Repository Update Script
# This script automatically adds, commits, and pushes changes to the repository

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${YELLOW}=== SAFEMRS Repository Update ===${NC}\n"

# Navigate to the repository directory
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$REPO_DIR" || exit 1

# Check if there are any changes
if [[ -z $(git status -s) ]]; then
    echo -e "${YELLOW}No changes to commit.${NC}"
    exit 0
fi

# Show status
echo -e "${YELLOW}Current changes:${NC}"
git status -s
echo ""

# Add all changes
echo -e "${GREEN}Adding all changes...${NC}"
git add .

# Get commit message from user or use default
if [ -z "$1" ]; then
    COMMIT_MSG="Update: $(date '+%Y-%m-%d %H:%M:%S')"
    echo -e "${YELLOW}No commit message provided. Using default:${NC} $COMMIT_MSG"
else
    COMMIT_MSG="$1"
fi

# Commit changes
echo -e "${GREEN}Committing changes...${NC}"
git commit -m "$COMMIT_MSG"

if [ $? -ne 0 ]; then
    echo -e "${RED}Commit failed!${NC}"
    exit 1
fi

# Push to remote
echo -e "${GREEN}Pushing to remote repository...${NC}"
git push origin main

if [ $? -ne 0 ]; then
    echo -e "${RED}Push failed! You may need to pull first or set upstream.${NC}"
    echo -e "${YELLOW}Try running: git push -u origin main${NC}"
    exit 1
fi

echo -e "\n${GREEN}✓ Successfully updated repository!${NC}"
