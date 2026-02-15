# GitHub Setup Instructions

## Initial Push Failed - Authentication Required

The initial push to https://github.com/asmbatati/SAFEMRS.git failed due to authentication.

## Solution Options

### Option 1: Use Personal Access Token (Recommended)

1. **Generate a Personal Access Token**:
   - Go to GitHub: https://github.com/settings/tokens
   - Click "Generate new token" → "Generate new token (classic)"
   - Give it a name: "SAFEMRS Repository Access"
   - Select scope: `repo` (Full control of private repositories)
   - Click "Generate token"
   - **COPY THE TOKEN IMMEDIATELY** (you won't see it again!)

2. **Update the remote URL with your token**:
   ```bash
   cd "/Users/akoubaa/Dropbox/ScaleX_Dev/PROJECT_PROPOSALS/RESEARCH_SUPERVISIONS/SAFEMRS IROS Paper"
   git remote set-url origin https://YOUR_GITHUB_TOKEN@github.com/asmbatati/SAFEMRS.git
   ```
   Replace `YOUR_GITHUB_TOKEN` with the token you just copied.

3. **Push to GitHub**:
   ```bash
   git push -u origin main
   ```

### Option 2: Use SSH Keys

1. **Generate SSH key** (if you don't have one):
   ```bash
   ssh-keygen -t ed25519 -C "your_email@example.com"
   ```

2. **Add SSH key to GitHub**:
   ```bash
   cat ~/.ssh/id_ed25519.pub
   ```
   Copy the output and add it to GitHub: https://github.com/settings/keys

3. **Update the remote URL**:
   ```bash
   cd "/Users/akoubaa/Dropbox/ScaleX_Dev/PROJECT_PROPOSALS/RESEARCH_SUPERVISIONS/SAFEMRS IROS Paper"
   git remote set-url origin git@github.com:asmbatati/SAFEMRS.git
   ```

4. **Push to GitHub**:
   ```bash
   git push -u origin main
   ```

### Option 3: Use GitHub CLI

1. **Install GitHub CLI**:
   ```bash
   brew install gh
   ```

2. **Authenticate**:
   ```bash
   gh auth login
   ```
   Follow the prompts to authenticate.

3. **Push to GitHub**:
   ```bash
   cd "/Users/akoubaa/Dropbox/ScaleX_Dev/PROJECT_PROPOSALS/RESEARCH_SUPERVISIONS/SAFEMRS IROS Paper"
   git push -u origin main
   ```

## After Authentication

Once authentication is set up, you can use the `update_repo.sh` script:

```bash
./update_repo.sh "Your commit message"
```

This will automatically add, commit, and push your changes.

## Verify Remote Configuration

To check your current remote configuration:
```bash
git remote -v
```

## Current Status

- ✅ Git repository initialized
- ✅ Remote added: https://github.com/asmbatati/SAFEMRS.git
- ✅ Initial commit created
- ⏳ Waiting for authentication setup
- ⏳ Pending: Push to remote

## Next Steps

1. Choose an authentication method above
2. Set up authentication
3. Run: `git push -u origin main`
4. Verify at: https://github.com/asmbatati/SAFEMRS

---

**Note**: Make sure you have permission to push to the `asmbatati/SAFEMRS` repository. If you don't own this repository, you may need to:
- Fork it to your own account, or
- Ask the repository owner to add you as a collaborator
