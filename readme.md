downloading conda enivronment

https://www.anaconda.com/download open this like 
![alt text](image.png)

open above image after opening the link you will get downloader option click on it

## 📦 Installation

### Step 1: Create Conda Environment

```bash
# Create new conda environment with Python 3.10+
conda create -n wire-analyzer python=3.10

# Activate environment
conda activate wire-analyzer
```

### Step 2: Install pythonocc-core

```bash
# Install pythonocc-core from conda-forge
conda install -c conda-forge pythonocc-core
```

### Step 3: Install Additional Dependencies

```bash
# Install numpy and matplotlib
pip install numpy matplotlib
```

### Verify Installation

```bash
python -c "from OCC.Core.STEPControl import STEPControl_Reader; print('✓ pythonocc-core installed successfully')"
```

---

## 📁 Project Structure

first create file name with "step_analyzer_occ.py"

- Creates `step_samples_occ/` directory inside it place your step files

```
.

├── step_analyzer_occ.py       # Analyze STEP files and extract geometry
├── step_samples_occ/          # Generated sample STEP files
├── analysis_results/          # Analysis output text files

```

---


### 2. Analyze a STEP File

```bash
# Analyze with results saved to file
python step_analyzer_occ.py step_samples_occ/3_bend_wire.step 

# Quick analysis (no file output)
python step_analyzer_occ.py step_samples_occ/3_bend_wire.step

after runing above commands  you will see structure like this >>>

```
.

├── step_analyzer_occ.py       # Analyze STEP files and extract geometry
├── step_samples_occ/          # Generated sample STEP files
├── analysis_results/          # Analysis output text files

```




