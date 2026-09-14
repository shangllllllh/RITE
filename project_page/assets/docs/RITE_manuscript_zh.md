# RITE：面向视觉退化端到端驾驶的反事实规划遗憾驱动迭代任务等价潜表征校正

**英文题名：** RITE: Regret-Guided Iterative Task-Equivalent Latent Correction for End-to-End Driving under Visual Degradation

> ICRA 2027 中文工作稿（双匿名）。章节按 8 页完整投稿的信息预算组织；最终页数以英文转写并套用 IEEE 双栏模板后的排版结果为准。方括号内容为待补充图表的占位符。

## 摘要

视觉退化会扰动端到端驾驶模型的潜表征，但表征偏差并不等同于规划风险，直接逼近完整干净特征还可能引入与驾驶决策无关的校正。本文提出 RITE（Regret-Guided Iterative Task-Equivalent Latent Correction），一种反事实规划遗憾驱动的迭代任务等价潜表征校正框架，将退化恢复分解为校正位置、执行路径和终点几何三个相互关联的问题。RITE 将单 token 沿配对干净方向干预所带来的规划损失下降定义为反事实规划遗憾，并据此学习选择性门控；通过中间轨迹监督与相邻步骤单调惩罚约束四步校正路径；依据冻结规划器的局部 Jacobian 构造最小范数任务等价终点。在 nuScenes 的严重混合视觉退化条件下，RITE 将平均规划 L2 从 0.6181 m 降低至 0.5573 m，并在保持轨迹收益的同时将实际校正能量较无终点约束变体降低 62.6%。在 NAVSIM v2 上，该方法将严重混合退化条件下的 EPDMS 相对 LAW 提高 22.18%，接入 SparseDriveV2 后在相同退化条件下也取得一致改善。结果表明，校正位置、迭代路径与潜空间干预规模可以在统一的规划任务几何下协同建模。

关键词：端到端自动驾驶；视觉退化；潜表征校正；反事实规划遗憾；任务等价校正

## I. 引言

端到端自动驾驶以统一模型将传感器观测映射为自车轨迹或控制量，使中间表征能够围绕最终驾驶目标进行学习。基于 nuScenes [1] 的视觉规划研究已从显式多任务协同发展到查询式场景建模、生成式轨迹预测和动作条件未来推演。UniAD 通过统一查询接口连接感知、预测与规划 [2]，VAD 以向量化场景元素提高规划表示效率 [3]，GenAD 在结构化潜空间中描述交通演化与多模态决策 [4]；潜世界模型则进一步利用未来表征支持轨迹生成和评价 [5], [6]。这些方法不断提高公开基准上的规划性能，也使潜表征成为视觉证据、动态先验与驾驶决策之间的关键接口。

公开基准主要描述标称成像分布，而实际驾驶中的视觉质量随天气、光照、运动状态和局部环境持续变化。雾、低照度、运动模糊、遮挡和眩光会以不同范围和强度破坏多视角证据，并通过时序融合影响后续状态。RoboBEV 和 3D Common Corruptions 的系统评测表明，干净条件下的 BEV 性能不足以代表模型在天气扰动、成像异常或传感器故障下的可靠性 [7], [8]。驾驶系统还会在同一行程中连续遭遇多种退化，因此，面向单一退化或单一强度建立的适配关系难以覆盖真实部署中的组合变化。

现有鲁棒性研究主要沿三条路径展开：在输入端增强或恢复受损图像，利用多模态冗余缓解单一传感器失效，或在特征空间重建受损的 BEV 表征；RESBev 等方法还利用历史语义状态补全当前观测 [9]。这些方法改善了退化条件下的图像质量、感知输出或密集特征一致性，但这些目标与规划效用并不完全一致。端到端驾驶的潜空间同时编码道路结构、动态目标、导航意图和数据分布先验，退化引起的表征差异中只有一部分会改变最终轨迹。若将所有偏差等同处理，恢复器既可能在规划无关维度消耗校正幅度，也可能把冻结规划器推离其经过验证的输入邻域。

由此，面向视觉退化的潜表征校正需要回答三个相互关联的问题。首先，校正位置应由下游任务效应决定；观测表征与时序先验的差异既可能来自退化，也可能对应真实场景运动，特征距离本身无法判断某个 token 是否值得干预。其次，迭代校正具有路径依赖性；各步更新连续作用于规划输入，仅约束最终状态不能保证中间状态沿有利于规划的方向演化。最后，能够产生相近轨迹变化的潜空间改动通常并不唯一；缺少任务敏感方向和尺度约束时，恢复器可能依赖大幅残差获得有限收益。与此同时，nuScenes open-loop L2 还可能受到自车状态和数据先验影响 [10], [11]，NeuroNCAP 也表明标称 open-loop 指标不足以刻画安全关键响应 [12]。因此，除最终轨迹误差和碰撞率外，还需要检验校正位置的反事实规划效用、多步路径的一致性以及取得给定规划收益所需的潜空间干预规模。

本文将视觉退化下的恢复表述为受规划任务几何约束的序贯潜表征校正问题，并提出 RITE（Regret-Guided Iterative Task-Equivalent Latent Correction）。RITE 保持基础视觉编码器、时序模型和规划器冻结，以单 token 沿配对干净方向干预后可避免的规划损失定义反事实规划遗憾，并据此学习选择性门控；规划一致循环校正完整展开部署时的四步更新，对中间状态和相邻步骤施加任务约束；最小范数任务等价目标则利用规划器局部 Jacobian 限制终点方向与尺度。三个设计分别处理校正的“位置—路径—终点”，训练期使用的干净表征、真实轨迹和 Jacobian 均不进入推理前向。

本文贡献如下：

1. 面向多类型、多强度视觉退化下的端到端规划，提出可插拔的 RITE 框架。在冻结基础规划器的条件下，RITE 将退化恢复重述为任务等价潜表征校正，并统一建模校正位置、迭代路径与终点尺度。
2. 提出反事实规划遗憾门控和规划一致循环校正：前者以一阶方向导数近似逐 token 反事实干预，获得任务级定位监督；后者通过中间轨迹监督与相邻步骤单调惩罚，约束四步校正路径。
3. 提出最小范数任务等价校正，在规划器的局部任务敏感子空间中构造低范数终点。nuScenes、NAVSIM v2 和 SparseDriveV2 上的实验从规划精度、碰撞、路径一致性和校正能量等方面验证了方法的有效性及跨规划架构适用性。

## II. 相关工作

### A. 视觉端到端规划与潜世界模型

视觉端到端驾驶的核心变化，是将场景理解和运动决策由串联模块转化为可联合优化的表示学习问题。ST-P3 通过时空 BEV 特征连接感知、预测与规划 [13]；TCP 将轨迹预测与控制预测结合，以未来轨迹为控制分支提供逐步引导 [14]。随后，ThinkTwice 通过条件未来想象和级联解码逐步细化轨迹 [15]，DriveAdapter 则利用特征对齐缓解感知教师与规划学生之间的耦合 [16]。在 nuScenes 上，UniAD、VAD 和 GenAD 分别代表统一查询、多任务向量表示与生成式规划的主要路线 [2]–[4]；SparseDriveV2 进一步通过因子化路径—速度词表与粗到细评分扩大候选轨迹覆盖 [33]。这类方法共同说明，规划性能取决于中间表征如何组织任务相关证据，而不仅是前端感知精度。

世界模型为端到端驾驶进一步引入了动作条件的未来推演。MILE 在紧凑潜空间中联合学习环境动态和驾驶策略 [17]；LAW 与 World4Drive 则把潜世界状态直接用于未来表征预测、候选轨迹生成或评价 [5], [6]。与上述工作不同，本文不训练新的驾驶策略或生成新的未来场景，而是将既有动作条件潜状态视为退化观测的时序参照，在冻结规划器前执行局部校正。研究重点由“如何预测未来”转向“如何判断当前潜表征中的哪些偏差会损害既有规划，并以受控方式加以修正”。

### B. 视觉退化鲁棒性与任务驱动恢复

多视角视觉驾驶依赖空间投影和跨时间融合，因而对成像退化与传感器缺失具有结构性敏感。BEVFormer 通过空间交叉注意力和时间自注意力构建统一 BEV 表征 [18]，为后续时序 BEV 方法提供了基础。面向传感器故障，BEVFusion 通过相对独立的相机与 LiDAR 分支提高融合系统在 LiDAR 异常下的可用性 [19]；MetaBEV 利用可选择聚合不同模态的查询处理传感器缺失 [20]。这些多模态方法依靠冗余传感器维持感知输出，而本文关注仅有相机输入且规划器已冻结时，能否利用模型内部的时间先验修正规划 token。

另一类工作从评测与恢复角度研究退化。在自动驾驶领域，RoboBEV、3D Common Corruptions、BEV 鲁棒性分析与 Robo3D 分别揭示相机 BEV、3D 检测及点云模型在天气、模糊、相机缺失和传感器噪声下的性能变化 [7], [8], [21], [22]；这些评测沿用了按扰动类型和严重度组织测试条件的通用 corruption 范式 [23]。RESBev 进一步利用历史语义状态重建受损 BEV 表征 [9]。这些研究主要以感知指标或密集特征质量为目标。本文则把恢复目标移至规划查询空间，并通过下游轨迹对 token 的反事实响应确定校正优先级，因而不要求恢复所有可见退化或完整重建干净 BEV。

通用图像恢复通常在像素空间去除特定或混合退化。Restormer 以高效 Transformer 建模高分辨率图像中的长程关系 [24]；TransWeather 以统一编码器—解码器处理多种恶劣天气 [25]；PromptIR 则利用退化提示调节同一恢复网络，以覆盖不同退化类型和强度 [26]。这类方法的主要目标是提高恢复图像与干净图像之间的保真度，其输出还需经过完整视觉主干才能影响驾驶决策。

任务驱动恢复开始显式考虑恢复结果对高层视觉任务的价值。UniRestore 区分感知质量与任务效用，并通过特征适配连接扩散先验和下游任务 [27]；EDTR 进一步限制扩散恢复过程，以保留对识别任务有用的细节 [28]。这些工作说明，视觉质量最优和任务性能最优并非同一目标。本文沿用这一任务导向，但不在像素空间生成视觉上合理的图像，也不以分类或分割特征作为代理目标，而是直接依据冻结规划器的损失变化学习选择性潜表征校正。

### C. 选择性迭代校正与规划评价

对全部 token 施加同等计算或同等改动通常并非必要。DynamicViT 通过输入相关的重要性预测逐层裁剪冗余 token [29]，说明 token 选择可以形成精度与计算量之间的动态权衡；在端到端驾驶中，ThinkTwice 的级联解码也表明多步细化能够扩展规划解码器的表达能力 [15]。然而，现有 token 选择多以最终预测准确率或计算效率为目标，多步细化也不必然约束每个中间状态的规划效应。RITE 将这两类思想分别改写为反事实规划效用和路径级任务一致性。

可微优化层为神经网络中显式表达约束提供了另一条路线。OptNet 展示了如何将参数化二次规划嵌入端到端模型并通过最优性条件传播梯度 [30]。RITE 不在推理时执行通用优化，而是在训练阶段利用冻结规划器的局部 Jacobian 求解低维正则化最小二乘问题，以构造任务等价的最小范数教师目标。由此，选择位置、多步路径和终点几何在同一恢复框架中分别获得可验证的约束。

nuScenes [1] 通常以未来轨迹 L2 与碰撞率评价 open-loop 规划。近期研究指出，仅使用 ego status 或显著削弱视觉输入的模型仍可能获得有竞争力的 L2 [10], [11]；NeuroNCAP 则从闭环安全关键场景侧面揭示了标称 open-loop 排名与交互响应之间的差异 [12]。NAVSIM 通过在真实数据上展开非反应式仿真，将碰撞、可行驶区域、进度和舒适性等规划属性纳入统一评价 [31]；NAVSIM v2 进一步以伪仿真协议和扩展预测驾驶模型得分（EPDMS）扩展评价维度 [32]。因此，本文保留 nuScenes L2 和官方 ego-box collision 作为可比主指标，同时增加四类机制证据，并在 NAVSIM v2 上检验跨数据集方法迁移及综合规划表现：精确 token 反事实替换检验定位有效性，逐步轨迹检验恢复路径，实际残差能量刻画潜空间干预规模，等 token／等能量干预检验收益是否来自所选择的位置与方向。

## III. 方法

### A. 问题定义与总体框架

给定时刻 $t$ 的六路相机观测 $\mathcal X_t$，其中任意视角可能受到未知类型和强度的视觉退化，目标是预测未来 $H=6$ 个时刻的二维自车位移 $\hat{\mathbf Y}_t\in\mathbb R^{H\times2}$。冻结视觉编码器与查询聚合器将观测压缩为

$$
\mathbf Z_t^o=\mathcal A(\mathcal E(\mathcal X_t))
\in\mathbb R^{N\times d},\qquad N=36,\ d=256.
$$

动作条件潜世界模型提供时序先验 $\mathbf Z_t^p$。可训练模块仅包括规划遗憾估计器 $D_\theta$ 与迭代潜表征校正器 $R_\psi$。完整恢复过程为

$$
(\mathbf s_t,\mathbf g_t)=D_\theta(\mathbf Z_t^o,\mathbf Z_t^p),\qquad
\mathbf Z_t^r=R_\psi(\mathbf Z_t^o,\mathbf Z_t^p,\mathbf g_t),
$$

冻结规划器 $F$ 根据 $\mathbf Z_t^r$ 输出轨迹。训练阶段额外使用配对干净表征 $\mathbf Z_t^c$ 和真实未来轨迹；推理阶段只需当前观测、时序先验、遗憾估计器和四步校正器。

这里，$\mathbf Z_t^o$、$\mathbf Z_t^p$ 和 $\mathbf Z_t^c$ 具有相同的 token 排列与通道维度。$\mathbf Z_t^c$ 由同一帧未施加合成退化的图像得到，仅用于构造训练监督；$\mathbf Z_t^p$ 由历史状态与自车动作条件产生，在训练和推理阶段均可获得。36 个 token 是基线查询聚合器形成的规划表示，不被解释为规则的 $6\times6$ 空间网格。本文只在该规划瓶颈处注入残差，从而避免改动高成本视觉编码器，也使每次校正都能通过同一冻结规划器进行任务评估。

**模块命名。** RITE 包含三个互补模块：反事实规划遗憾门控（Counterfactual Planning Regret Gating, **CPRG**）、规划一致循环校正（Planning-Consistent Recurrent Refinement, **PCRR**）和最小范数任务等价校正（Minimum-norm Task-Equivalent Correction, **MTEC**）。后文以模块缩写的加法组合准确表示消融配置：CPRG 表示仅启用遗憾门控，CPRG+MTEC 表示启用遗憾门控与任务等价终点约束，CPRG+PCRR 表示启用遗憾门控与多步路径监督；**RITE 始终专指 CPRG+PCRR+MTEC 的完整模型**。

总体计算顺序为：首先以遗憾门控估计各 token 的规划影响并形成连续门控；随后由循环校正器以共享参数执行四次有界残差更新；最后，训练期的任务等价教师约束第四步实际注入残差。门控输出在四个步骤间保持不变，使“校正哪些位置”和“如何更新这些位置”在建模上相互分离；路径监督与终点监督分别约束沿途状态和最终状态，避免用单一重建损失同时承担定位、路径与尺度控制。

> **图 1 占位：RITE 总体框架。** 左侧为冻结基础规划器生成观测 token 与动作条件时序先验；中部自上而下展示反事实规划遗憾门控（CPRG）、四步规划一致循环校正（PCRR）和最小范数任务等价终点（MTEC）；右侧为门控残差注入与冻结规划头。训练专用的 clean/GT/Jacobian 路径用虚线，部署路径用实线。

### B. 反事实规划遗憾门控

表征距离无法区分规划相关变化与无关变化。我们以单 token 干预后的规划损失下降定义精确遗憾。令 $\mathbf Z_t^{(i\leftarrow c)}$ 表示仅将观测表征中的第 $i$ 个 token 替换为配对干净 token，则

$$
r_{t,i}^{\mathrm{exact}}=
\mathcal L_{\mathrm{plan}}(F(\mathbf Z_t^o),\mathbf Y_t)
-\mathcal L_{\mathrm{plan}}(F(\mathbf Z_t^{(i\leftarrow c)}),\mathbf Y_t).
$$

精确计算需要每个样本执行 $N$ 次规划前向。训练时在 $\mathbf Z_t^o$ 处采用一阶近似：

$$
r_{t,i}^{\mathrm{grad}}=
\left[-\left\langle
\frac{\partial\mathcal L_{\mathrm{plan}}}{\partial\mathbf z_{t,i}^o},
\mathbf z_{t,i}^c-\mathbf z_{t,i}^o
\right\rangle\right]_+ .
$$

以样本内最大正遗憾归一化得到 $\tilde r_{t,i}$。目标与规划梯度均停止梯度，因此不产生二阶导数。部署遗憾分数由

$$
s_{t,i}=\sigma\!\left(f_\theta[\mathbf z_{t,i}^o;
\mathbf z_{t,i}^p;|\mathbf z_{t,i}^o-\mathbf z_{t,i}^p|]\right)
$$

预测，并通过

$$
g_{t,i}=\sigma\big(\kappa(s_{t,i}-\tau)\big)
$$

形成连续 token 门控。本文采用 $\tau=0.35$、$\kappa=6$，遗憾监督为

$$
\mathcal L_{\mathrm{reg}}=
\mathrm{SmoothL1}(\mathbf s_t,\mathrm{sg}(\tilde{\mathbf r}_t)).
$$

该分数表示样本内相对规划影响，而非物理退化概率。

上述定义包含两层约束。首先，干预方向固定为 $\mathbf z_{t,i}^c-\mathbf z_{t,i}^o$，因此遗憾衡量的是“沿配对干净方向移动该 token 是否降低规划损失”，而不是无约束的梯度幅值。其次，正部算子丢弃一阶估计为负的替换方向，避免把“更像干净特征但可能增大当前规划损失”的 token 当作正监督。样本内归一化只保留同一场景中的相对排序，降低不同场景轨迹尺度和有效航点数量对回归目标的影响。

在部署阶段，$D_\theta$ 无法访问 $\mathbf Z_t^c$ 或真实轨迹，因此以观测 token、时序先验及二者的绝对差作为输入。时序差异提供退化候选线索，观测内容与先验内容则帮助估计器区分真实动态变化和不一致观测。连续门控 $g_{t,i}$ 不执行硬删除：较低分 token 仍保留小幅更新通道，较高分 token 获得更大的残差注入比例。这一设计既便于端到端训练，也为后续 Top／Random／Bottom 等预算干预提供统一的排序变量。

### C. 规划一致循环校正

多步潜表征校正由一系列相互依赖的中间状态构成，仅监督最终状态无法刻画各步更新对下游规划的影响。为使完整执行路径接受任务约束，循环校正器在训练阶段显式展开部署所用的 $K=4$ 步校正。令 $\tilde{\mathbf Z}^{(0)}_t=\mathbf Z_t^o$，第 $k$ 步更新为

$$
\tilde{\mathbf Z}^{(k)}_t=\tilde{\mathbf Z}^{(k-1)}_t+
\frac{1}{K}R_\psi(\tilde{\mathbf Z}^{(k-1)}_t,
\mathbf Z_t^p,u_k).
$$

每一步相对观测的候选残差经有界映射和遗憾门控后得到

$$
\mathbf d_t^{(k)}=\mathbf g_t\odot
\alpha\tanh(\tilde{\mathbf Z}^{(k)}_t-\mathbf Z_t^o),\qquad
\mathbf Z_t^{r,(k)}=\mathbf Z_t^o+\mathbf d_t^{(k)},
$$

其中 $\alpha=0.3$。冻结规划器在第 $k$ 步产生的监督轨迹损失记为

$$
\ell_k=\mathcal L_{3D}(F(\mathbf Z_t^{r,(k)}),\mathbf Y_t,\mathbf M_t),
$$

$\mathbf M_t$ 为未来有效掩码，$\ell_0$ 由观测表征计算并停止梯度。本文对前三个中间状态施加路径监督：

$$
\mathcal L_{\mathrm{path}}=\frac{1}{K-1}\sum_{k=1}^{K-1}\ell_k,
$$

并惩罚任意一步相对前一步的规划损失上升：

$$
\mathcal L_{\mathrm{mono}}=\frac{1}{K}\sum_{k=1}^{K}
\left[\ell_k-\mathrm{sg}(\ell_{k-1})\right]_+ .
$$

最终一步仍由原有轨迹与任务一致性目标监督。该设计将任务监督从恢复终点扩展到离散执行路径，使中间状态及相邻步骤间的规划变化成为可训练对象。

$u_k$ 为步骤嵌入，用于区分不同校正阶段；$1/K$ 的步长缩放使单步候选更新与总步数解耦。每一步均重新以当前中间状态和固定时序先验计算候选残差，但实际注入始终相对于原始观测 $\mathbf Z_t^o$ 构造。这样可以把残差幅度限制在统一坐标系内，避免连续的增量累加绕过 $\tanh$ 边界。遗憾门控在所有步骤共享，因此循环校正器主要学习被选 token 的更新轨迹，而不会在中间步骤隐式改变定位标准。

$\mathcal L_{\mathrm{path}}$ 与 $\mathcal L_{\mathrm{mono}}$ 承担不同作用。前者使前三个中间状态本身具备可规划性，防止优化只在第四步形成有效表示；后者比较相邻状态，直接惩罚相对前一步的任务损失回升。停止梯度的 $\ell_{k-1}$ 仅作为局部参照，避免后一步的惩罚反向推动前一步损失增大。该约束提高的是统计意义上的路径一致性，而不是对每个样本给出严格单调保证；因此实验同时报告逐步均值、相邻非增比例、四步全程非增比例以及终点相对路径最优点的差值。

### D. 最小范数任务等价校正

路径监督可持续降低轨迹误差，但若缺少潜空间尺度约束，恢复器可能通过扩大残差获得收益。为此，我们在观测表征处线性化冻结规划器：

$$
F(\mathbf Z_t^o+\boldsymbol\delta)\approx
F(\mathbf Z_t^o)+\mathbf J_t\boldsymbol\delta,
\qquad
\mathbf J_t=\left.\frac{\partial F}{\partial\mathrm{vec}(\mathbf Z)}
\right|_{\mathbf Z_t^o}.
$$

令干净与退化分支的规划差为

$$
\Delta\boldsymbol\tau_t=
\mathrm{sg}(F(\mathbf Z_t^c))-F(\mathbf Z_t^o)\in\mathbb R^{12}.
$$

我们将终点教师目标定义为正则化最小范数任务等价校正：

$$
\boldsymbol\delta_t^\star=
\arg\min_{\boldsymbol\delta}
\frac{1}{2}\|\mathbf J_t\boldsymbol\delta-
\Delta\boldsymbol\tau_t\|_2^2+
\frac{\mu_t}{2}\|\boldsymbol\delta\|_2^2,
$$

其对偶闭式解为

$$
\boldsymbol\delta_t^\star=
\mathbf J_t^\top(\mathbf J_t\mathbf J_t^\top+
\mu_t\mathbf I)^{-1}\Delta\boldsymbol\tau_t.
$$

由于轨迹输出仅为 12 维，实现只需求解 $12\times12$ 线性系统，而无需构造潜空间逆矩阵。$\mu_t$ 采用 Jacobian Gram 矩阵迹的尺度自适应正则，教师目标逐元素截断到 $[-0.3,0.3]$。该解位于 $\mathrm{range}(\mathbf J_t^\top)$，从而排除不改变局部规划输出却增加范数的零空间分量。任务等价目标直接监督第四步实际注入残差：

$$
\mathcal L_{\mathrm{MTEC}}=\mathrm{SmoothL1}
(\mathbf Z_t^{r,(K)}-\mathbf Z_t^o,
\mathrm{sg}(\boldsymbol\delta_t^\star)).
$$

Jacobian、干净轨迹和线性求解仅用于训练。推理时 RITE 仍执行遗憾门控和四步校正，不增加优化步骤或 Jacobian 计算。

该目标中的“任务等价”是局部意义上的：在规划器的一阶近似下，$\mathbf J_t\boldsymbol\delta_t^\star$ 尽可能复现干净与退化分支之间的轨迹变化 $\Delta\boldsymbol\tau_t$，而不要求 $\mathbf Z_t^o+\boldsymbol\delta_t^\star$ 重建完整干净表征。由于潜表征维度 $Nd=9216$ 远大于轨迹输出维度 12，同一轨迹变化通常对应大量潜空间解。二次范数项在这些近似任务等价解中偏向较小改动，并以 $\mu_t$ 调节轨迹复现误差与干预规模之间的权衡。

这一“预算约束”并非仅为限制本方法的残差数值。从学术上看，它把退化恢复由全表征重建转化为欠定任务逆问题：研究对象不再是唯一的干净潜变量，而是能产生等价下游行为的一组解，并通过最小范数原则选取局部代表。从工程上看，冻结规划器只在训练分布附近经过验证，较小的潜空间位移相当于对部署接口施加局部信赖域，减少恢复模块把规划器推向未校准区域的机会。同时，校正能量和 token 数可以作为独立于 L2 的干预预算，使不同方法能够在相同资源或相同扰动规模下进行比较。

### E. 训练目标

完整目标为

$$
\mathcal L=\mathcal L_{\mathrm{base}}+
\lambda_r\mathcal L_{\mathrm{reg}}+
\lambda_p\mathcal L_{\mathrm{path}}+
\lambda_m\mathcal L_{\mathrm{mono}}+
\lambda_{\mathrm{MTEC}}\mathcal L_{\mathrm{MTEC}},
$$

其中 $\mathcal L_{\mathrm{base}}$ 包含基线规划、世界模型、潜表征恢复、噪声方向、干净旁路和任务一致性目标。本文采用 $\lambda_r=1$、$\lambda_p=0.1$、$\lambda_m=0.3$、$\lambda_{\mathrm{MTEC}}=3$。视觉主干、查询聚合器、世界模型和规划头保持冻结，仅更新遗憾估计器和潜表征校正器。遗憾门控决定校正位置，循环校正约束执行路径，任务等价目标约束终点方向与尺度；三个目标分别作用于不同变量和阶段。

训练时，退化分支与干净分支共享冻结的视觉编码器和规划器。遗憾门控所需的规划梯度以及任务等价目标所需的 Jacobian 均在当前 mini-batch 内计算，其输出作为停止梯度的教师信号；优化器不会通过这些教师构造过程更新冻结模块。推理时首先计算一次 $D_\theta$，随后顺序执行四步 $R_\psi$ 并只将第四步表征送入规划头。因而，训练期增加的干净分支、真实未来轨迹、反向梯度和线性系统求解均被移除，部署路径只保留遗憾估计与共享参数的迭代校正。

**算法 1：RITE 的训练与部署流程**

**输入：**退化／干净图像对 $(\mathcal X_t^o,\mathcal X_t^c)$，时序先验 $\mathbf Z_t^p$，真实轨迹 $\mathbf Y_t$；冻结的编码器、查询聚合器和规划器；迭代步数 $K=4$。

1. 计算观测与干净 token：$\mathbf Z_t^o,\mathbf Z_t^c$。
2. 由一阶方向导数构造停止梯度的遗憾教师，并由 $D_\theta$ 预测共享门控 $\mathbf g_t$。
3. 初始化 $\tilde{\mathbf Z}_t^{(0)}=\mathbf Z_t^o$ 和观测规划损失 $\ell_0$。
4. 对 $k=1,\ldots,K$，执行共享校正器，形成有界门控残差 $\mathbf d_t^{(k)}$，并计算中间轨迹损失 $\ell_k$。
5. 由 $\{\ell_k\}_{k=0}^{K}$ 计算 $\mathcal L_{\mathrm{path}}$ 与 $\mathcal L_{\mathrm{mono}}$。
6. 计算冻结规划器的局部 Jacobian，并求得停止梯度的任务等价终点 $\boldsymbol\delta_t^\star$。
7. 组合完整训练目标，仅更新 $D_\theta$ 与 $R_\psi$。
8. 推理时移除干净分支、真实轨迹和 Jacobian 求解，预测一次门控并执行四步校正，输出 $F(\mathbf Z_t^{r,(K)})$。

## IV. 实验

### A. 数据、退化协议与实现细节

实验基于 nuScenes [1]。验证集包含 150 个场景和 6,019 个样本，其中 5,119 个样本具有完整有效的未来轨迹。训练阶段以 0.5 概率对六路相机施加雾、低照度、运动模糊、局部遮挡或眩光，严重度 1／2／3 的采样权重为 0.20／0.35／0.45。主测试条件记为 Hard-S3，对全部相机施加由上述五类退化构成的严重度 3 混合扰动；Clean 保留原始图像，用于衡量恢复模块对标称条件的影响。扩展鲁棒性矩阵还包括五类单独 S3、Mixed S1／S2／S3、单前视相机退化、随机单相机退化、前视相机 blackout、全相机 blackout 和错配时序先验。

> **退化协议示例图。** 六幅图取自同一时刻的 CAM_FRONT：Clean、Fog-S3、Low-light-S3、Motion blur-S3、Occlusion-S3 与 Glare-S3。论文所用图片已集中存放在 `figures/corruption_examples/`。S3 表示本文协议中的最高退化强度，Hard-S3 从五类 S3 退化中构造多相机混合扰动。

跨数据集评价首先采用 NAVSIM v2 [31], [32] 的 `fprt-dev-val` 划分，共含 10,106 个 token 和 112 个 log。NAVSIM 接口输出 8 个未来位姿，时间间隔为 0.5 s、预测时域为 4 s。我们在 NAVSIM 上完成 LAW 的域内训练，冻结其视觉主干、潜世界模型和规划头，仅训练 RITE 恢复模块 12 epoch；该设置检验方法架构和监督机制的跨数据集迁移能力。

为进一步考察恢复机制对基础规划架构的依赖，我们将 RITE 接入 SparseDriveV2 [33]。该实验冻结官方 SparseDriveV2 基础网络，引入固定的动作条件潜世界模型先验，并训练四步潜表征恢复模块 12 epoch；部署时只执行恢复前向，不计算 Jacobian、VJP 或训练损失。SparseDriveV2 评价覆盖 12,146 个 token，原模型与 RITE 增强模型在 Clean／Hard-S3 下均为 12,146 次成功、0 次失败。两组 NAVSIM 实验的 Hard-S3 均对全部相机施加雾、夜间、运动模糊、遮挡和眩光五类严重度 3 退化，并采用一致的退化配置。

nuScenes 主实验采用 LAW [5] 的公开设置，包括 Swin-Tiny 主干、36 个 256 维规划 token 和 6 个二维未来航点。所有恢复变体训练 12 epoch，优化器为 AdamW，初始学习率为 $10^{-4}$，weight decay 为 0.01，有效全局 batch size 为 4，最大梯度范数为 35。RITE 的三个子机制使用相同的数据顺序、增强分布和冻结策略，模型差异仅来自相应损失项。完整模型采用 $\lambda_{\mathrm{MTEC}}=3$，权重敏感性实验另取 $\lambda_{\mathrm{MTEC}}\in\{2,30\}$。

nuScenes 指标为 1、2、3 s 平均规划 L2（m）和官方 ego-box collision（%）。校正规模以 actual applied residual energy 衡量，即实际注入残差的样本均方能量；同时记录门控前 candidate energy，以区分候选更新收缩和门控选择的作用。不同变体在完全一致的样本集合和评价实现下比较，分布式采样产生的重复样本在统计前去重。NAVSIM v2 使用修正 GT 碰撞底噪的统一 scorer，报告 EPDMS 及 no-at-fault collision（NC）、drivable area compliance（DAC）、time-to-collision（TTC）、two-frame extended comfort（Comfort-2F）和 ego progress（EP）等子项。LAW-NAVSIM 的四个模型—条件分支均完成 10,106／10,106 个有效 token 评分，SparseDriveV2 的四个分支均完成 12,146／12,146 个有效 token 评分。

### B. 对比方法与实验问题

实验设置包含两个层次。首先，在相同 LAW 主干、图像输入、数据划分和评价器下，比较原始规划器与逐步引入三个机制的模型，分析 RITE 的整体增益及各组件的作用。其次，在 NAVSIM v2 上分别以 LAW 和 SparseDriveV2 为基础规划器，对比各自的原模型与 RITE 增强模型，检验方法能否跨数据集和跨规划架构工作。由于 nuScenes 与 NAVSIM v2 的轨迹接口和指标不同，两组结果分别报告，不进行跨协议数值排序。

消融链包含五个模型：LAW 为不含恢复模块的冻结规划器，其余四种配置均按照实际启用的模块命名。为便于阅读，这里再次说明：CPRG 为仅含反事实规划遗憾门控的配置，CPRG+MTEC 在此基础上加入最小范数任务等价校正，CPRG+PCRR 则加入规划一致循环校正，RITE 为同时启用三者的完整模型。全文均采用上述组件组合式命名。任务等价损失权重 $\lambda_{\mathrm{MTEC}}\in\{2,3,30\}$ 只用于敏感性与精度—能量 Pareto，其中 RITE 固定取 3。

实验首先报告整体性能与组件贡献，随后围绕四个机制与泛化问题展开：(1) 一阶反事实遗憾能否近似精确 token 干预，并使门控在等预算下选择更有规划价值的位置；(2) 规划一致循环校正是否改善四步执行路径，而不仅是最终状态的平均 L2；(3) 最小范数任务等价校正能否在保留规划收益的同时压缩循环校正的残差能量，并形成可解释的 token 预算 Pareto；(4) 上述结论能否扩展至不同退化条件、数据集和基础规划架构。

### C. 主要结果与组件贡献

表 I 首先比较 LAW、三个组件组合以及完整 RITE。RITE 在 Clean 和 Hard-S3 上的规划 L2 分别为 0.5582 m 和 0.5573 m，相比 LAW 的 0.6214 m 和 0.6181 m 分别降低 0.0632 m（10.17%）和 0.0608 m（9.84%）。这一结果表明，RITE 在标称条件和严重混合退化下均能稳定改善冻结规划器的轨迹预测。

逐组件结果揭示了三个设计的分工。CPRG 在 Clean／Hard-S3 上取得 0.5726／0.5703 m 的规划 L2，相比 LAW 分别降低 0.0488／0.0478 m，说明以规划遗憾定位关键 token 的校正配置能够显著改善轨迹预测；其定位能力将在后续等预算干预实验中进一步验证。在 CPRG 上加入规划一致循环校正得到 CPRG+PCRR，其 L2 进一步降至 0.5544／0.5547 m，构成表中精度最优的工作点，说明对中间状态和相邻更新施加规划约束能够持续改善多步校正路径。CPRG+MTEC 的 L2 与 CPRG 基本一致，同时将 Hard-S3 碰撞率由 0.2691% 降至 0.2556%。完整 RITE 将三种机制联合起来，取得 0.5582／0.5573 m 的 L2；结合后续能量分析，其 Clean／Hard-S3 实际校正能量较 CPRG+PCRR 分别降低约 58.5%／62.6%，形成兼顾轨迹精度与潜空间干预规模的工作点。

碰撞指标呈现与规划 L2 互补的结果。RITE 将 Clean／Hard-S3 的官方 Box Collision 从 LAW 的 0.2534%／0.3283% 降至 0.2214%／0.2621%，相对降幅分别为 12.63%／20.16%。其中，RITE 在 Clean 条件下取得最低碰撞率，CPRG+PCRR 在 Hard-S3 下取得最低碰撞率；这说明轨迹距离与离散碰撞事件关注不同的误差区域，完整模型在显著改善轨迹精度的同时保持了整体碰撞收益。

**表 I  nuScenes Clean 与 Hard-S3 主要结果及组件消融。**

| 方法 | Clean L2 ↓ | Clean Box Col. (%) ↓ | Hard-S3 L2 ↓ | Hard-S3 Box Col. (%) ↓ |
| --- | ---: | ---: | ---: | ---: |
| LAW [5] | 0.6214 | 0.2534 | 0.6181 | 0.3283 |
| CPRG | 0.5726 | 0.2241 | 0.5703 | 0.2691 |
| CPRG+MTEC | 0.5723 | 0.2274 | 0.5697 | 0.2556 |
| CPRG+PCRR | **0.5544** | 0.2252 | **0.5547** | **0.2491** |
| RITE（完整） | 0.5582 | **0.2214** | 0.5573 | 0.2621 |

为减少数据记录中 GT 轨迹自身碰撞对规划器评价的影响，我们还报告逐时刻剔除该部分后的 corrected collision。相对 LAW，RITE 在 Clean／Hard-S3 上分别将 corrected collision frames 从 154／218 降至 126／147，对应 corrected frame rate 从 0.5014%／0.7098% 降至 0.4102%／0.4786%，相对下降 18.18%／32.57%。结合组件结果可见，反事实规划遗憾门控为碰撞改善提供主要贡献，而规划一致循环校正和任务等价终点约束分别强化轨迹质量与校正规模控制。

### D. 反事实遗憾的近似质量与选择性

我们从 150 个验证场景各固定选取 4 个有效帧，共得到 600 个 Hard-S3 样本，并对每个样本的 36 个 token 逐一执行 clean 替换。表 II 显示，一阶遗憾与精确遗憾具有一致的样本内排序。图 2(a) 的横轴为精确遗憾的归一化 token 排名，纵轴为一阶遗憾排名；六边形密度集中在对角线附近，说明一阶近似能够保留精确干预的相对次序。相应的场景级 Spearman 相关为 0.8274，Top-25% overlap 为 0.7778；按一阶遗憾选出的顶部 token，其精确遗憾也显著高于底部和随机 token。图 2(b) 进一步以横轴给出不同退化下的场景级 Spearman 系数，五类退化的结果均处于 0.801–0.860，且围绕总体水平分布，表明这种排序一致性并非由单一退化类型主导。一次规划反向传播由此能够替代每个样本 36 次独立替换前向，为遗憾估计器提供可扩展监督。

**表 II  一阶遗憾与精确 token 反事实影响。**

| 指标 | 场景级估计 |
| --- | ---: |
| Spearman($r^{\mathrm{grad}},r^{\mathrm{exact}}$) | **0.8274** |
| Top-25% overlap | **0.7778** |
| 正遗憾符号准确率 | **0.9203** |
| Top–Bottom exact-regret 差 | **0.01998** |
| Top–Random exact-regret 差 | **0.01096** |

精确 target 的一致性并不等同于部署估计器已经学会选择。为此，我们在 RITE checkpoint 上进行等预算干预。图 3(c) 的横轴列出不同 token 选择策略，纵轴为 Hard-S3 L2，虚线表示不施加校正的 Observed 结果。在相同的 25% token 预算下，Top-regret 分支取得 0.57019 m，分别优于 Random-25% 的 0.60409 m 和 Bottom-25% 的 0.61599 m，对应差值为 -0.03390 m 和 -0.04580 m；完整 learned gate 相对等能量 shuffled gate 的 L2 降低 0.05072 m。该结果将遗憾监督的证据由“target 可近似”扩展到“预测排序具有实际规划效用”。

> **图 2　反事实遗憾定位和多步路径机制分析。** (a) 精确遗憾排名与一阶遗憾排名的 token 密度；(b) 不同退化类型下的场景级 Spearman 相关；(c) 四种组件配置随校正步数变化的 Hard-S3 规划损失；(d) Step-4 applied energy 与四步全程非增率，左上方向表示以更低能量获得更稳定的校正路径。

### E. 规划一致循环校正的多步路径一致性

首先考察缺少路径监督时的多步行为。固定 CPRG+MTEC（不含 PCRR）checkpoint 在 600 个样本上由一步扩展到四步后，平均 L2 增加 0.0141 m，且 92.33% 样本的最优步数不是固定的第四步。这一现象表明，终点重建或单步有效并不能推出重复执行后的任务收益，多步路径需要直接监督。

随后将路径分析扩展至全部 6,019 个验证样本。表 III 显示，引入规划一致循环校正后，Clean／Hard-S3 的四步全程非增比例由 48.99%／39.05% 提高到 55.28%／42.18%，最终 L2 从 0.57232／0.56973 m 降至 0.55440／0.55473 m。图 2(c) 的横轴表示从观测状态 $\ell_0$ 到第四次校正 $\ell_4$ 的执行步数，纵轴为平均规划损失。CPRG 和 CPRG+MTEC 在前两步下降后较快趋于平缓，而 CPRG+PCRR 与 RITE 的曲线在后续步骤仍持续下降；其中，路径监督使第四步 Hard-S3 规划损失由 CPRG+MTEC 的 0.20894 降至 0.20504。进一步加入最小范数任务等价校正后，Hard-S3 全程非增比例达到 45.91%。图 2(d) 以 Step-4 applied energy 为横轴、四步全程非增率为纵轴，因而越靠近左上方表示能量越低且路径越稳定。图中从 CPRG+PCRR 指向 RITE 的变化表明，MTEC 将能量从 0.8411 压缩至 0.3152，同时将全程非增率从 42.18% 提高到 45.91%，即能量降低 62.53%、稳定路径比例提高 3.7 个百分点，而最终 L2 仅增加 0.00253 m。这些统计量说明，路径监督的作用体现为分布层面的路径一致性与平均终点收益提升，MTEC 则进一步控制达到该路径收益所需的潜空间干预规模。

**表 III  完整验证集上的四步路径机制分析。**

| 方法 | 条件 | $\ell_0$ | $\ell_4$ | $\ell_4-\ell_0$ ↓ | 全程非增率 ↑ | $\ell_4-\min_k\ell_k$ ↓ | Step-4 L2 ↓ | Step-4 energy ↓ |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| CPRG+MTEC | Clean | 0.222179 | 0.211071 | -0.011108 | 48.99% | 0.004590 | 0.572320 | 0.035865 |
| CPRG+MTEC | Hard-S3 | 0.222518 | 0.208939 | -0.013579 | 39.05% | 0.009710 | 0.569725 | 0.182070 |
| CPRG+PCRR | Clean | 0.222179 | **0.206103** | **-0.016077** | 55.28% | 0.005820 | **0.554402** | 0.160497 |
| CPRG+PCRR | Hard-S3 | 0.222518 | **0.205042** | **-0.017476** | 42.18% | 0.011464 | **0.554733** | 0.841127 |
| RITE | Clean | 0.222179 | 0.207625 | -0.014554 | **55.85%** | 0.005517 | 0.558187 | **0.066547** |
| RITE | Hard-S3 | 0.222518 | 0.205815 | -0.016703 | **45.91%** | 0.010440 | 0.557266 | **0.315155** |

### F. 最小范数任务等价校正与预算 Pareto

表 IV 分离了最小范数任务等价校正对规划精度和残差能量的作用。在仅使用遗憾门控的变体上加入该约束后，Hard-S3 L2 保持在相近水平，applied energy 由 0.2162 降至 0.1809。规划一致循环校正将 Hard-S3 L2 降至 0.5547 m，同时将 energy 增至 0.8409；RITE 将 energy 压缩到 0.3146，较无终点约束变体降低约 62.6%，同时保留 83.7% 的路径校正 L2 收益。Clean 条件下呈现相同关系：RITE 的 energy 为 0.0664，较无终点约束变体降低约 58.5%。图 3(a) 以 applied energy 为横轴、Hard-S3 L2 为纵轴，理想工作点位于左下方。CPRG+PCRR 位于低 L2、高能量区域；随着 $\lambda_{\mathrm{MTEC}}$ 从 2 增大到 30，RITE 工作点向低能量方向移动并伴随一定精度变化，其中本文采用的 $\lambda_{\mathrm{MTEC}}=3$ 位于精度与能量之间的折中区域。图 3(d) 进一步给出全部样本的能量累积分布，横轴为对数尺度下的逐样本 applied energy，纵轴为累计样本比例。RITE 曲线相对 CPRG+PCRR 整体左移，中位能量由 0.781 降至 0.262，说明平均能量降低 62.6% 并非由少量极端样本造成，而是覆盖了大部分验证样本。

**表 IV  组件作用、规划收益与实际校正能量。**

| 方法 | CPRG | PCRR | MTEC | Clean energy ↓ | Hard-S3 energy ↓ | Hard-S3 L2 相对 CPRG ↓ |
| --- | :---: | :---: | :---: | ---: | ---: | ---: |
| CPRG | ✓ | — | — | 0.0366 | 0.2162 | — |
| CPRG+MTEC | ✓ | — | ✓ | **0.0356** | **0.1809** | -0.0005 m |
| CPRG+PCRR | ✓ | ✓ | — | 0.1601 | 0.8409 | **-0.0155 m** |
| RITE | ✓ | ✓ | ✓ | 0.0664 | 0.3146 | -0.0130 m |

token 预算实验进一步考察收益是否集中于少量高遗憾位置。图 3(b) 的横轴为激活 token 的比例，纵轴同时给出相对全量校正的 L2 收益保留率和能量使用比例。对采用 $\lambda_{\mathrm{MTEC}}=3$ 的 RITE 按遗憾分数保留 $K=0/4/9/18/36$ 个 token 时，收益曲线在低预算区间上升更快：$K=4/9/18$ 分别使用完整校正 52.2%／71.9%／86.8% 的能量，并保留 68.8%／80.4%／89.7% 的 Hard-S3 L2 收益；相应 L2 为 0.57784／0.57019／0.56407，完整 $K=36$ 端点为 0.55727。尤其在 $K=9$ 时，仅激活 25% 的 token 即可保留 80.4% 的全量收益，说明规划收益主要集中于高遗憾位置，但 token 稀疏并不等同于等比例的能量稀疏。$\lambda_{\mathrm{MTEC}}=2$ 得到相近曲线，说明该 Pareto 趋势并非默认权重的偶然结果；$\lambda_{\mathrm{MTEC}}=30$ 仅作为更强能量约束端点展示，不用于主结果。

> **图 3　精度—能量与 token 预算 Pareto。** (a) 不同组件和 MTEC 权重的 Hard-S3 L2—applied energy 工作点；(b) 不同激活 token 预算下的收益保留率与能量比例；(c) 25% token 等预算选择策略以及 Observed 和完整门控参照；(d) CPRG+PCRR 与 RITE 的逐样本 applied energy 累积分布。

### G. 跨退化、效率与定性分析

跨退化实验在 13 个条件下统一评估 CPRG+PCRR 和 RITE：五类单独 S3、Mixed S1／S2／S3、两类部分相机退化、两类 blackout 和 shuffled prior。每个条件包含 6,019 个样本，其中 5,119 个 future-valid 样本覆盖 150 个场景。表 V 汇总各条件组的 L2。对 12 个时序先验匹配条件，两种模型均稳定降低 LAW 的规划误差；L2 宏平均分别由 0.62746 m 降至 0.55173 m 和 0.55437 m，即降低 12.07% 和 11.65%。CPRG+PCRR 的 L2 点估计略低，而完整路径分析显示，加入 MTEC 的 RITE 仅以很小的精度差异显著压缩校正能量。

**表 V  跨退化鲁棒性矩阵（L2）。**

| 条件组 | 条件数 | LAW | CPRG+PCRR | RITE |
| --- | ---: | ---: | ---: | ---: |
| 五类单退化 S3（宏平均） | 5 | 0.6204 | **0.5525** | 0.5550 |
| 强度与局部视角（宏平均） | 5 | 0.6182 | **0.5510** | 0.5540 |
| CAM_FRONT／ALL_CAM blackout（宏平均） | 2 | 0.6682 | **0.5515** | 0.5536 |
| Shuffled temporal prior | 1 | **0.6181** | 0.7203 | 0.7111 |

作为时序先验敏感性测试，shuffled prior 会使 RITE 的 applied energy 升至 0.6951，约为其余 12 个条件宏平均的 3.11 倍，表明校正能量能够反映时序参照与当前观测之间的异常错配。

效率评估在同一 RTX 3090、batch size 1 和固定精度模式下进行，预热 100 个样本后测量 1,000 个样本。遗憾监督所需的规划梯度、任务等价教师所需的 12 行 Jacobian 和 $12\times12$ 线性求解均属于训练期开销，不计入部署；四步共享恢复器完整计入推理延迟。表 VI 显示，RITE 增加 0.821 M 参数（2.14%）、0.373 supported GFLOPs（0.07%）和约 0.004 GiB 峰值显存。RITE 与 LAW 的 model-only mean latency 分别为 145.47 ms 和 147.10 ms；不同权重设置下的重复测试表明，部署延迟变化保持在约 0–4% 范围内。

**表 VI  完整模型部署效率（RTX 3090，batch size 1）。**

| 方法 | 总参数（新增） | Supported GFLOPs | Model mean／P95 | E2E FPS | Peak allocated |
| --- | ---: | ---: | ---: | ---: | ---: |
| LAW | 38.333 M（—） | 542.276 | 147.10／168.83 ms | 3.900 | 0.652 GiB |
| RITE | 39.154 M（+0.821 M） | 542.649 | 145.47／169.72 ms | 3.903 | 0.656 GiB |

定性图从预先规定的成功、中性和失败集合中选取样本，展示六相机退化输入、36-token 遗憾排序、观测／四步恢复／最终／GT 轨迹以及逐步 L2 和 applied energy。token 仅按查询索引可视化，不绘制为虚构的 $6\times6$ 空间热图。blackout 和 shuffled-prior 样本单独用于说明当视觉证据或时间参照失效时的恢复边界。

### H. NAVSIM v2 跨数据集与跨规划器评价

**LAW-NAVSIM 规划器。** 为检验方法是否依赖 nuScenes 特定的规划接口和 open-loop L2 目标，我们首先在 NAVSIM 上训练 LAW，并在相同基础模型上训练 RITE。两者采用相同的 10,106 个 `fprt-dev-val` token、112 个 log、轨迹导出实现、Hard-S3 退化配置和统一 scorer。RITE 从 LAW 权重初始化后仅训练恢复模块，以评价方法架构和监督机制的跨数据集迁移能力。

**SparseDriveV2 规划器。** 我们进一步在冻结的 SparseDriveV2 基础网络上接入 RITE。原模型与增强模型使用完全相同的 12,146-token 集合、NAVSIM v2 metric cache 和 Hard-S3 参数；RITE 额外读取固定动作条件潜世界模型先验，并以阈值 0.35 执行四步校正。下表在各自相同数据与评价协议内报告两组配对结果。

**表 VII  RITE 在 NAVSIM v2 上的跨规划器评价结果。每组只比较同一基础模型在接入 RITE 前后的变化。**

| 方法 | 条件 | NC ↑ | DAC ↑ | TTC ↑ | Comfort-2F ↑ | EP ↑ | EPDMS ↑ |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| LAW | Clean | 0.960172 | 0.916089 | 0.946368 | 0.619892 | **0.888323** | 0.798826 |
| LAW + RITE | Clean | **0.962547** | **0.917475** | **0.948051** | **0.632954** | 0.886536 | **0.802923** |
| LAW | Hard-S3 | 0.726549 | 0.782604 | 0.689887 | 0.191132 | **0.890385** | 0.454067 |
| LAW + RITE | Hard-S3 | **0.801603** | **0.830299** | **0.772313** | **0.291672** | 0.878351 | **0.554762** |
| SparseDriveV2 | Clean | 0.985139 | **0.981805** | 0.978676 | **0.780876** | **0.904334** | **0.903878** |
| SparseDriveV2 + RITE | Clean | **0.985674** | 0.980817 | **0.978923** | 0.778685 | 0.903133 | 0.903008 |
| SparseDriveV2 | Hard-S3 | 0.944591 | 0.898897 | 0.933229 | 0.601594 | 0.828060 | 0.759819 |
| SparseDriveV2 + RITE | Hard-S3 | **0.948913** | **0.910012** | **0.935617** | **0.605279** | **0.846397** | **0.776775** |

在 LAW-NAVSIM 上，RITE 在 Clean 条件下将 EPDMS 从 0.798826 提高到 0.802923，基本保持标称条件性能；Hard-S3 下则从 0.454067 提高到 0.554762，相对提高 22.18%，同时改善 NC、DAC、TTC 和 Comfort-2F，Final L2 由 8.182788 m 降至 6.565949 m。在 SparseDriveV2 上，RITE 在 Clean 条件下基本保持 EPDMS（0.903878 对 0.903008），并将 Hard-S3 EPDMS 从 0.759819 提高到 0.776775，相对提高 2.23%；该条件下 NC、DAC、TTC、Comfort-2F 和 EP 均得到改善。两组配对结果表明，RITE 的退化恢复作用能够迁移至不同的数据集评价协议与基础规划架构。

## V. 结论

本文提出 RITE，将退化视觉下的潜表征恢复分解为“位置—路径—终点”三个任务相关问题：反事实规划遗憾用于选择值得校正的 token，路径级监督约束四步迭代过程，最小范数任务等价目标限制最终校正的方向与尺度。该设计不依赖重新训练基础规划器，训练期使用的干净表征、真实轨迹和 Jacobian 也不会进入部署前向。

nuScenes 实验表明，RITE 在 Clean 和 Hard-S3 条件下均降低了平均规划 L2；等预算干预验证了高遗憾 token 的规划效用，路径监督改善了多步校正的一致性，任务等价目标则在保留大部分轨迹收益的同时将 Hard-S3 校正能量压缩 62.6%。跨退化实验和 NAVSIM v2 评价进一步显示，RITE 能够在不同退化条件、数据集和基础规划架构上保持稳定收益，并仅引入 2.14% 的额外参数量。

这些结果说明，视觉退化恢复无需以完整干净特征为唯一目标，而可以在下游任务等价的潜空间解中选择规划有效且干预受控的校正。当前方法仍依赖可靠的时序先验；后续将进一步研究先验错配下的校正拒绝机制，并在真实恶劣天气和反应式闭环环境中验证其稳定性。

## 参考文献

[1] H. Caesar, V. Bankiti, A. H. Lang, et al., “nuScenes: A Multimodal Dataset for Autonomous Driving,” in *Proc. IEEE/CVF CVPR*, pp. 11621–11631, 2020.

[2] Y. Hu, J. Yang, L. Chen, et al., “Planning-Oriented Autonomous Driving,” in *Proc. IEEE/CVF CVPR*, pp. 17853–17862, 2023.

[3] B. Jiang, S. Chen, Q. Xu, et al., “VAD: Vectorized Scene Representation for Efficient Autonomous Driving,” in *Proc. IEEE/CVF ICCV*, pp. 8340–8350, 2023.

[4] W. Zheng, R. Song, X. Guo, C. Zhang, and L. Chen, “GenAD: Generative End-to-End Autonomous Driving,” in *Proc. ECCV*, 2024.

[5] Y. Li, L. Fan, J. He, et al., “Enhancing End-to-End Autonomous Driving with Latent World Model,” in *Proc. ICLR*, 2025.

[6] Y. Zheng, P. Yang, Z. Xing, et al., “World4Drive: End-to-End Autonomous Driving via Intention-Aware Physical Latent World Model,” in *Proc. IEEE/CVF ICCV*, pp. 28632–28642, 2025.

[7] S. Xie, L. Kong, W. Zhang, et al., “RoboBEV: Towards Robust Bird’s Eye View Perception under Corruptions,” arXiv:2304.06719, 2023.

[8] Y. Dong, C. Kang, J. Zhang, et al., “Benchmarking Robustness of 3D Object Detection to Common Corruptions,” in *Proc. IEEE/CVF CVPR*, pp. 1022–1032, 2023.

[9] L. Zhuo, K. Jin, Z. Liu, and H. Wang, “RESBev: Making BEV Perception More Robust,” arXiv:2603.09529, 2026.

[10] Z. Li, Z. Yu, S. Lan, et al., “Is Ego Status All You Need for Open-Loop End-to-End Autonomous Driving?” in *Proc. IEEE/CVF CVPR*, pp. 14864–14873, 2024.

[11] J.-T. Zhai, Z. Feng, J. Du, et al., “Rethinking the Open-Loop Evaluation of End-to-End Autonomous Driving in nuScenes,” arXiv:2305.10430, 2023.

[12] W. Ljungbergh, A. Tonderski, J. Johnander, et al., “NeuroNCAP: Photorealistic Closed-Loop Safety Testing for Autonomous Driving,” in *Proc. ECCV*, 2024.

[13] S. Hu, L. Chen, P. Wu, H. Li, J. Yan, and D. Tao, “ST-P3: End-to-End Vision-Based Autonomous Driving via Spatial-Temporal Feature Learning,” in *Proc. ECCV*, 2022.

[14] P. Wu, X. Jia, L. Chen, J. Yan, H. Li, and Y. Qiao, “Trajectory-Guided Control Prediction for End-to-End Autonomous Driving: A Simple yet Strong Baseline,” in *Adv. Neural Inf. Process. Syst.*, vol. 35, 2022.

[15] X. Jia, P. Wu, L. Chen, et al., “Think Twice Before Driving: Towards Scalable Decoders for End-to-End Autonomous Driving,” in *Proc. IEEE/CVF CVPR*, pp. 21983–21994, 2023.

[16] X. Jia, Y. Gao, L. Chen, J. Yan, P. L. Liu, and H. Li, “DriveAdapter: Breaking the Coupling Barrier of Perception and Planning in End-to-End Autonomous Driving,” in *Proc. IEEE/CVF ICCV*, pp. 7953–7963, 2023.

[17] A. Hu, G. Corrado, N. Griffiths, et al., “Model-Based Imitation Learning for Urban Driving,” in *Adv. Neural Inf. Process. Syst.*, vol. 35, 2022.

[18] Z. Li, W. Wang, H. Li, et al., “BEVFormer: Learning Bird’s-Eye-View Representation from Multi-Camera Images via Spatiotemporal Transformers,” in *Proc. ECCV*, 2022.

[19] T. Liang, H. Xie, K. Yu, et al., “BEVFusion: A Simple and Robust LiDAR-Camera Fusion Framework,” in *Adv. Neural Inf. Process. Syst.*, vol. 35, 2022.

[20] C. Ge, J. Chen, E. Xie, et al., “MetaBEV: Solving Sensor Failures for 3D Detection and Map Segmentation,” in *Proc. IEEE/CVF ICCV*, pp. 8721–8731, 2023.

[21] Z. Zhu, Y. Zhang, H. Chen, et al., “Understanding the Robustness of 3D Object Detection With Bird’s-Eye-View Representations in Autonomous Driving,” in *Proc. IEEE/CVF CVPR*, pp. 21600–21610, 2023.

[22] L. Kong, Y. Liu, X. Li, et al., “Robo3D: Towards Robust and Reliable 3D Perception against Corruptions,” in *Proc. IEEE/CVF ICCV*, pp. 19994–20006, 2023.

[23] D. Hendrycks and T. Dietterich, “Benchmarking Neural Network Robustness to Common Corruptions and Perturbations,” in *Proc. ICLR*, 2019.

[24] S. W. Zamir, A. Arora, S. Khan, et al., “Restormer: Efficient Transformer for High-Resolution Image Restoration,” in *Proc. IEEE/CVF CVPR*, pp. 5728–5739, 2022.

[25] J. M. J. Valanarasu, R. Yasarla, and V. M. Patel, “TransWeather: Transformer-Based Restoration of Images Degraded by Adverse Weather Conditions,” in *Proc. IEEE/CVF CVPR*, pp. 2353–2363, 2022.

[26] V. Potlapalli, S. W. Zamir, S. Khan, and F. S. Khan, “PromptIR: Prompting for All-in-One Blind Image Restoration,” in *Adv. Neural Inf. Process. Syst.*, vol. 36, 2023.

[27] I.-H. Chen, W.-T. Chen, Y.-W. Liu, Y.-C. Chiang, S.-Y. Kuo, and M.-H. Yang, “UniRestore: Unified Perceptual and Task-Oriented Image Restoration Model Using Diffusion Prior,” in *Proc. IEEE/CVF CVPR*, pp. 17969–17979, 2025.

[28] J. Kim, J. Oh, and K. M. Lee, “Exploiting Diffusion Prior for Task-Driven Image Restoration,” in *Proc. IEEE/CVF ICCV*, pp. 10151–10161, 2025.

[29] Y. Rao, W. Zhao, B. Liu, J. Lu, J. Zhou, and C.-J. Hsieh, “DynamicViT: Efficient Vision Transformers with Dynamic Token Sparsification,” in *Adv. Neural Inf. Process. Syst.*, vol. 34, 2021.

[30] B. Amos and J. Z. Kolter, “OptNet: Differentiable Optimization as a Layer in Neural Networks,” in *Proc. ICML*, pp. 136–145, 2017.

[31] D. Dauner, M. Hallgarten, T. Li, et al., “NAVSIM: Data-Driven Non-Reactive Autonomous Vehicle Simulation and Benchmarking,” in *Adv. Neural Inf. Process. Syst.*, 2024.

[32] W. Cao, M. Hallgarten, T. Li, et al., “Pseudo-Simulation for Autonomous Driving,” in *Proc. Conf. Robot Learn.*, pp. 4709–4722, 2025.

[33] W. Sun, X. Lin, K. Chen, Z. Pei, X. Li, Y. Shi, and S. Zheng, “SparseDriveV2: Scoring is All You Need for End-to-End Autonomous Driving,” arXiv:2603.29163, 2026.
