#!/bin/bash

# Create GitHub Repository using API
# This script creates a new repository on GitHub using the GitHub API

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${YELLOW}=== Creating GitHub Repository ===${NC}\n"

# Check if GITHUB_TOKEN is set
if [ -z "$GITHUB_TOKEN" ]; then
    echo -e "${RED}Error: GITHUB_TOKEN environment variable is not set${NC}"
    exit 1
fi

# Repository details
REPO_NAME="SAFEMRS"
REPO_DESCRIPTION="Safe Agentic Framework for Externally-augmented Multi-Robot Systems - LLM-based multi-robot task planning with formal safety verification"
PRIVATE=false  # Set to true if you want a private repo

echo -e "${GREEN}Creating repository: $REPO_NAME${NC}"

# Create repository using GitHub API
RESPONSE=$(curl -s -w "\n%{http_code}" -X POST \
  -H "Accept: application/vnd.github+json" \
  -H "Authorization: Bearer $GITHUB_TOKEN" \
  -H "X-GitHub-Api-Version: 2022-11-28" \
  https://api.github.com/user/repos \
  -d "{\"name\":\"$REPO_NAME\",\"description\":\"$REPO_DESCRIPTION\",\"private\":$PRIVATE}")

# Extract HTTP status code
HTTP_CODE=$(echo "$RESPONSE" | tail -n1)
BODY=$(echo "$RESPONSE" | head -n -1)

if [ "$HTTP_CODE" -eq 201 ]; then
    echo -e "${GREEN}✓ Repository created successfully!${NC}"
    CLONE_URL=$(echo "$BODY" | grep -o '"clone_url": "[^"]*' | cut -d'"' -f4)
    HTML_URL=$(echo "$BODY" | grep -o '"html_url": "[^"]*' | cut -d'"' -f4)
    echo -e "${GREEN}Repository URL: $HTML_URL${NC}"
elif [ "$HTTP_CODE" -eq 422 ]; then
    echo -e "${YELLOW}Repository already exists. Continuing...${NC}"
    CLONE_URL="https://github.com/asmbatati/SAFEMRS.git"
else
    echo -e "${RED}Failed to create repository. HTTP Code: $HTTP_CODE${NC}"
    echo -e "${RED}Response: $BODY${NC}"
    exit 1
fi

echo -e "\n${GREEN}Next steps:${NC}"
echo "1. Repository is ready at: https://github.com/asmbatati/SAFEMRS"
echo "2. Pushing your code..."

# Update remote and push
cd "$(dirname "${BASH_SOURCE[0]}")"
git remote set-url origin "https://${GITHUB_TOKEN}@github.com/asmbatati/SAFEMRS.git"
git push -u origin main

if [ $? -eq 0 ]; then
    echo -e "\n${GREEN}✓ Successfully pushed to GitHub!${NC}"
    echo -e "${GREEN}Visit: https://github.com/asmbatati/SAFEMRS${NC}"
else
    echo -e "\n${RED}Push failed. Please check the error above.${NC}"
    exit 1
fi
