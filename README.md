## Experiment Code Description
This code implements comparative experiments on task scheduling under different load conditions for two scenarios: **Edge-cloud** and **Edge-only**. Key experimental metrics are outputted, including average task migration count, scheduling failure rate, and average scheduling cost.
## Experimental Tools
MATLAB 2022a.
### Algorithm Naming Convention
- Proposed algorithm: **MOSDT** (named `Algorithm1` in the code)
- Baseline algorithms for comparison:
   - DCDS
   - COFE
### Experiment File Mapping
| File Name       | Scenario          | Load Type       |
| :-------------- | :---------------- | :-------------- |
| `Compare1`      | Edge-cloud        | Low-load        |
| `Compare2`      | Edge-cloud        | Normal-load     |
| `Compare3`      | Edge-cloud        | Over-load       |
| `Edgecompare1`  | Edge-only         | Low-load        |
| `Edgecompare2`  | Edge-only         | Normal-load     |
| `Edgecompare3`  | Edge-only         | Over-load       |
### Experimental Metric Abbreviations
| Abbreviation | Definition                         |
| :----------- | :--------------------------------- |
| `mt`         | Average task migration numbers     |
| `failrate`   | Scheduling failure rate            |
| `c`          | Average scheduling cost            |
