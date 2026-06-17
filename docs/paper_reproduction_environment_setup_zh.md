# 论文复现项目环境配置说明

本文档用于把当前 `rpmi_v0` 项目的 Python 环境配置方式迁移到独立的论文复现项目中。目标是复用当前机器上已经跑通的工具链和基础依赖，同时避免论文复现实验污染 `rpmi_v0` 的源码、依赖和提交记录。

## 当前环境快照

当前项目采用 `uv` 管理本地虚拟环境，不直接依赖全局 Anaconda 环境。环境入口如下：

```text
项目路径: D:\PycharmProjects\rpmi_v0
环境目录: D:\PycharmProjects\rpmi_v0\.venv
解释器:   D:\PycharmProjects\rpmi_v0\.venv\Scripts\python.exe
Python:   3.12.12
uv:       0.6.14
```

`.venv` 的基础解释器来自本机 Anaconda 安装：

```text
base_prefix: D:\anaconda3
```

这表示当前项目虽然借用了 Anaconda 提供的 Python 3.12.12，但依赖安装和项目运行应走 `uv` 创建的项目内 `.venv`，不要直接在 `D:\anaconda3` 里安装论文复现依赖。

## 当前项目依赖策略

当前 `pyproject.toml` 的环境要求是：

```text
Python: >=3.12
runtime: numpy, pyyaml
dev: pytest
```

当前锁定并实际可用的核心版本如下：

| 包 | 当前版本 | 用途 |
| --- | --- | --- |
| `numpy` | `2.4.5` | 数值计算基础依赖 |
| `pyyaml` | `6.0.3` | YAML 配置读取 |
| `pytest` | `9.0.3` | 测试 |
| `colorama` | `0.4.6` | `pytest` 在 Windows 下的间接依赖 |
| `iniconfig` | `2.3.0` | `pytest` 间接依赖 |
| `packaging` | `26.2` | `pytest` 间接依赖 |
| `pluggy` | `1.6.0` | `pytest` 间接依赖 |
| `pygments` | `2.20.0` | `pytest` 间接依赖 |

当前项目没有声明 PyTorch、TensorFlow、CUDA、SUMO、MOBIL、强化学习框架或交通仿真依赖。论文复现如果需要这些依赖，应根据论文和官方安装说明在复现项目中单独增加，不要直接改动 `rpmi_v0` 的环境。

## 推荐原则

论文复现项目建议使用独立目录和独立 `.venv`：

```text
D:\PycharmProjects\paper_name_repro\.venv
```

不要直接复用：

```text
D:\PycharmProjects\rpmi_v0\.venv
```

原因是当前 `.venv` 里包含 `rpmi-v0` 这个可编辑安装的本地项目包，并且路径绑定到 `D:\PycharmProjects\rpmi_v0`。论文复现项目如果直接指向这个环境，后续安装论文依赖时容易影响当前项目，也容易把复现代码和 `rpmi_v0` 代码混在一起。

推荐做法是：

```text
复用 Python 版本和依赖策略，不复用同一个可写虚拟环境。
```

## 新复现项目的最小环境配置

在新复现项目中保留一份 `.python-version`：

```text
3.12
```

建议用 `pyproject.toml` 声明基础环境。以下配置适合作为论文复现项目的起点：

```toml
[project]
name = "paper-name-repro"
version = "0.1.0"
description = "Research paper reproduction workspace."
requires-python = ">=3.12"
dependencies = [
    "numpy>=1.26",
    "pyyaml>=6.0",
]

[project.optional-dependencies]
dev = [
    "pytest>=8.0",
]
```

如果论文需要额外依赖，例如 `torch`、`scipy`、`pandas`、`matplotlib`、`scikit-learn`、`transformers` 等，建议先跑通最小环境，再按论文需要逐项加入。不要一开始把 Anaconda 里的全部包冻结进复现项目。

## 创建或刷新环境

在新复现项目根目录执行：

```powershell
uv venv --python 3.12
uv sync --extra dev
```

不想手动激活环境时，统一通过 `uv run` 执行命令：

```powershell
uv run python --version
uv run pytest
```

需要进入交互式环境时再激活：

```powershell
.\.venv\Scripts\Activate.ps1
```

激活后仍建议优先用 `uv` 管理依赖，避免把包安装到全局 Anaconda。

## 环境验证

新复现项目环境创建完成后，先做最小验证：

```powershell
uv run python --version
uv run python -c "import sys, numpy, yaml; print(sys.executable); print(numpy.__version__); print(yaml.__version__)"
uv tree
```

预期方向：

```text
Python 版本应为 3.12.x
解释器路径应指向新复现项目自己的 .venv
numpy 和 pyyaml 应能正常 import
uv tree 应只展示新复现项目声明和解析出的依赖
```

如果项目配置了测试，优先跑：

```powershell
uv run pytest
```

在 Windows PowerShell 中，不建议依赖通配符展开测试文件，例如：

```powershell
uv run pytest tests/test_*.py
```

PowerShell 可能不会按预期展开通配符。更稳妥的方式是运行整个测试目录，或显式列出测试文件：

```powershell
uv run pytest tests
```

## 冻结当前环境作为参考

如果只是想把当前 `rpmi_v0` 的依赖解析结果保存为参考，不建议使用裸命令：

```powershell
uv pip freeze
```

在当前机器上，这个命令可能指向 `D:\anaconda3`，从而导出大量与 `rpmi_v0` 无关的 Anaconda 包。

推荐从当前项目的 `uv.lock` 导出一份干净的 requirements 参考：

```powershell
uv export --extra dev --no-hashes --no-emit-project --frozen --output-file requirements.rpmi-v0.baseline.txt
```

这份文件适合放进复现项目作为“当前环境基线参考”，但不一定应该长期作为论文复现项目的主依赖文件。论文复现项目的主依赖仍建议由自己的 `pyproject.toml` 和 `uv.lock` 管理。

如果确实要按这份 baseline 安装到新项目的 `.venv`，应显式指定新项目解释器：

```powershell
uv venv --python 3.12
uv pip install --python .\.venv\Scripts\python.exe -r requirements.rpmi-v0.baseline.txt
```

安装后检查：

```powershell
uv pip list --python .\.venv\Scripts\python.exe
```

## 添加论文依赖的建议

论文依赖应按以下顺序加入：

```text
1. 先保留当前基础环境: Python 3.12 + uv + numpy + pyyaml + pytest
2. 阅读论文或官方代码的依赖说明
3. 确认是否需要特定 Python / CUDA / PyTorch / TensorFlow 版本
4. 在复现项目中逐项添加依赖
5. 每添加一组关键依赖就运行一次最小验证
6. 能跑通小样例后再锁定 uv.lock
```

如果论文明确要求老版本 Python，例如 Python 3.8、3.9 或 3.10，不建议强行沿用当前 Python 3.12。此时应为论文单独创建匹配版本的环境，并在复现文档里记录原因。

如果论文需要 GPU 训练，不要默认认为当前 `rpmi_v0` 环境已经具备 CUDA/PyTorch 能力。应单独记录：

```text
GPU 型号:
显卡驱动版本:
CUDA 版本:
PyTorch / TensorFlow 版本:
安装命令:
验证命令:
```

## 常见注意事项

`uv sync --extra dev` 过程中可能出现旧 wheel 的 `Skipping file` 警告。只要命令最终成功退出，一般可以继续。

当前项目的 `.venv` 没有安装 `pip` 模块，因此不要依赖：

```powershell
uv run python -m pip freeze
```

需要查看项目环境包时，使用：

```powershell
uv tree
uv pip list --python .\.venv\Scripts\python.exe
```

需要导出锁定依赖时，使用：

```powershell
uv export --extra dev --no-hashes --no-emit-project --frozen --output-file requirements.txt
```

论文复现依赖不要写回 `rpmi_v0` 的 `pyproject.toml`。如果论文复现项目和 `rpmi_v0` 无代码关系，应让复现项目拥有自己的 `pyproject.toml`、`uv.lock` 和 `.venv`。

## 推荐工作流

从当前环境迁移到新论文复现环境时，建议按这个顺序做：

```text
1. 在 D:\PycharmProjects 下新建独立复现项目目录
2. 写入 .python-version，使用 Python 3.12
3. 写入最小 pyproject.toml
4. 执行 uv venv --python 3.12
5. 执行 uv sync --extra dev
6. 执行 uv run python --version 和 uv tree
7. 根据论文逐项添加额外依赖
8. 每次关键依赖变更后提交或记录环境变更
9. 跑通最小样例后再进行完整实验
```

一句话总结：

```text
按当前项目复用 Python 3.12、uv 和基础依赖策略；为论文复现项目新建独立 .venv；依赖由复现项目自己锁定和演进。
```
