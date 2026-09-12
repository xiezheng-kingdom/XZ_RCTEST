# XZ_RCTEST
谢峥的RC战队G308考核
答案在分支里

---

## 二自由度云台控制板（分支 `RCTEST_YUNTAI_CONTROL`）

STM32H743，两个电位器或一个 MPU6050 产生云台目标角度，PE3 按键切换输入源，
角度以 `偏航,俯仰` 文本经 USART3 上报。

- 架构设计、踩坑记录、标定方法、当前验证状态 → [docs/DEVELOPMENT.md](docs/DEVELOPMENT.md)
