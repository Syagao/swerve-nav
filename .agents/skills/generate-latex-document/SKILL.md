---
name: generate-latex-document
description: "使用内置 LaTeX 模板，将 Markdown、TXT 等零散资料整理成技术文档，或者分析代码库生成结构化代码说明文档。仅在用户显式调用 $generate-latex-document 时使用。"
---

# 生成 LaTeX 技术文档

使用 Skill 内置模板生成完整、独立、可直接上传 Overleaf 的 `.tex` 文档。

## 确定工作模式

根据用户请求选择一种模式：

- 笔记整理模式：处理 Markdown、TXT、会议记录、学习笔记和其他零散资料。
- 代码库说明模式：分析项目代码，生成模块、文件、类、函数和接口说明。

如果无法判断模式，先询问用户。

选择模式后只读取对应工作流：

- 笔记整理模式：读取 `references/notes-workflow.md`。
- 代码库说明模式：读取 `references/codebase-workflow.md`。

不要同时读取两个工作流，除非用户明确要求混合生成。

## 使用模板

读取 `assets/document-template.tex`，复制为新的输出文件，不修改原始模板。

替换以下占位符：

- `CODEXDOCUMENTTITLE`：文档标题；
- `CODEXDOCUMENTSUBTITLE`：文档副标题；
- `CODEXPROJECTNAME`：项目或主题名称；
- `CODEXDOCUMENTAUTHOR`：作者；
- `CODEXDOCUMENTVERSION`：文档版本；
- `CODEXDOCUMENTDESCRIPTION`：封面简介；
- `CODEXDOCUMENTBODY`：完整正文。

如果用户没有提供作者，使用“项目组”或沿用已有文档作者，不得猜测真实姓名。

如果用户没有提供版本，首次生成使用 `v1.0`；更新已有文档时沿用原版本，除非用户要求修改。

## 输出要求

只生成完整的 `.tex` 文件，不生成 PDF。

不调用本地 LaTeX、XeLaTeX、latexmk 或其他编译工具，也不检查本地是否安装 LaTeX。

输出文件必须：

- 包含完整导言区、封面、目录、正文和 `\end{document}`；
- 能够独立上传 Overleaf；
- 不依赖 Skill 目录中的模板；
- 不保留任何 `CODEX...` 占位符；
- 不保留 `XXXX`、示例数据或无意义空章节；
- 默认适配 Overleaf 的 XeLaTeX 编译器；
- 保留模板的主要视觉风格。

默认不要使用外部 `.sty`、`\input{}` 或 `\include{}`。

如果文档需要图片，可以使用相对路径，并报告需要同时上传的图片文件。

## LaTeX 内容规则

正确处理 LaTeX 特殊字符：

```text
%  _  &  #  $  {  }  ~  ^  \