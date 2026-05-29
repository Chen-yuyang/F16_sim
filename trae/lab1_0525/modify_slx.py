"""
修改 F16_openloop.slx，在 Trimmed Input 子系统中加入升降舵 Doublet 扰动注入。

新增模块：
  - Sum_Disturb (SID 103): 将 de 配平值与扰动信号相加
  - Elev_Disturb (SID 104): FromWorkspace，从 base workspace 读取 elev_dist_data

重新连线：
  de (70) ──→ Sum_Disturb (103) ──→ Elevator act. (52)
                ↑
  Elev_Disturb (104)

使用方法: python modify_slx.py
"""

import zipfile
import os

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
try:
    _proj = os.path.abspath(os.path.join(SCRIPT_DIR, '..', '..', '..'))
    _proj.encode('ascii')
    PROJ_ROOT = _proj
    SRC_SLX = os.path.join(PROJ_ROOT, 'F16_openloop.slx')
except (UnicodeEncodeError, UnicodeDecodeError):
    # Windows 中文路径兼容: 基于当前工作目录
    PROJ_ROOT = os.getcwd()
    SRC_SLX = os.path.join(PROJ_ROOT, 'F16_openloop.slx')
DST_SLX = os.path.join(SCRIPT_DIR, 'F16_openloop_dist.slx')


def main():
    with zipfile.ZipFile(SRC_SLX, 'r') as z:
        files = {name: z.read(name) for name in z.namelist()}

    sys43 = files['simulink/systems/system_43.xml'].decode('utf-8')

    # ── 2a. 在 Out1 块后插入 Sum + FromWorkspace 块 ──
    # 注意: 必须包含 <P Name=" 以匹配完整的属性元素开始，
    #       否则替换后会在 Out1 块内留下孤立的残缺标签
    out1_close = '<P Name="VectorParamsAs1DForOutWhenUnconnected">off</P>\n  </Block>'
    assert out1_close in sys43, "找不到 Out1 块结尾"

    new_blocks_xml = """  </Block>
  <Block BlockType="Sum" Name="Sum_Disturb" SID="103">
    <PortCounts in="2" out="1"/>
    <P Name="Position">[270, 215, 295, 235]</P>
    <P Name="ZOrder">-10</P>
    <P Name="ShowName">off</P>
    <P Name="Inputs">++</P>
    <P Name="OutDataTypeStr">Inherit: Same as first input</P>
    <P Name="InputSameDT">on</P>
    <P Name="SaturateOnIntegerOverflow">on</P>
  </Block>
  <Block BlockType="FromWorkspace" Name="Elev_Disturb" SID="104">
    <PortCounts in="0" out="1"/>
    <P Name="Position">[200, 280, 230, 310]</P>
    <P Name="ZOrder">-11</P>
    <P Name="VariableName">elev_dist_data</P>
    <P Name="SampleTime">0</P>
    <P Name="OutputAfterFinalValue">SettingToZero</P>
  </Block>"""

    out1_end_marker = out1_close + '\n  <Line>'
    sys43 = sys43.replace(out1_end_marker, new_blocks_xml + '\n  <Line>', 1)

    # ── 2b. 修改 de→Elevator act. 连线为 de→Sum_Disturb ──
    old_line = '''  <Line>
    <P Name="ZOrder">6</P>
    <P Name="Src">70#out:1</P>
    <P Name="Dst">52#in:1</P>
  </Line>'''

    new_line = '''  <Line>
    <P Name="ZOrder">6</P>
    <P Name="Src">70#out:1</P>
    <P Name="Dst">103#in:1</P>
  </Line>'''

    assert old_line in sys43, "找不到 de→Elevator act. 连线"
    sys43 = sys43.replace(old_line, new_line, 1)

    # ── 2c. 在 </System> 前添加 Sum→Elevator act. 和 FromWorkspace→Sum 连线 ──
    extra_lines = '''  <Line>
    <P Name="ZOrder">9</P>
    <P Name="Src">103#out:1</P>
    <P Name="Dst">52#in:1</P>
  </Line>
  <Line>
    <P Name="ZOrder">10</P>
    <P Name="Src">104#out:1</P>
    <P Name="Dst">103#in:2</P>
  </Line>
'''

    sys43 = sys43.replace('</System>', extra_lines + '</System>', 1)

    files['simulink/systems/system_43.xml'] = sys43.encode('utf-8')

    # ── 3. 写出新的 .slx ──
    with zipfile.ZipFile(DST_SLX, 'w', zipfile.ZIP_STORED) as z:
        for name, data in files.items():
            z.writestr(name, data)

    # ── 4. 完整性检查: 验证 XML 格式正确且连线完整 ──
    verify_xml(DST_SLX)

    print("[OK] 修改完成: %s" % DST_SLX)
    print("     新增 Sum_Disturb (SID 103), Elev_Disturb (SID 104)")
    print("     重新连线: de -> Sum_Disturb -> Elevator act.")


def verify_xml(path):
    """检查生成的 .slx 中的 system_43.xml 是否有明显问题"""
    import xml.etree.ElementTree as ET
    import re

    with zipfile.ZipFile(path, 'r') as z:
        sys43 = z.read('simulink/systems/system_43.xml').decode('utf-8')

    # 检查 XML 末尾是否包含多余的文本（常见错误）
    end_marker = '</System>'
    end_idx = sys43.find(end_marker)
    if end_idx >= 0:
        tail = sys43[end_idx + len(end_marker):]
        if tail.strip():
            print("[WARN] </System> 后有额外内容: %s" % repr(tail[:100]))

    # 检查所有 Line 元素是否有 Src 和 Dst
    lines = re.findall(r'<Line>.*?</Line>', sys43, re.DOTALL)
    for i, l in enumerate(lines):
        src = re.search(r'Src">(\d+)#out:(\d+)', l)
        dst = re.search(r'Dst">(\d+)#in:(\d+)', l)
        if not src or not dst:
            print("[WARN] Line %d: 缺少 Src 或 Dst" % i)
            continue

    # 验证没有孤立的残缺标签
    orphan = re.search(r'<P Name="\s*</', sys43)
    if orphan:
        print("[ERROR] 发现孤立残缺标签: ...%s..." % repr(sys43[max(0, orphan.start()-20):orphan.end()+20]))
        raise SystemExit(1)

    # 验证关键连线
    conn_checks = [
        (r'70#out:1[\s\S]*?103#in:1', '70->103 (de->Sum)'),
        (r'103#out:1[\s\S]*?52#in:1', '103->52 (Sum->act)'),
        (r'104#out:1[\s\S]*?103#in:2', '104->103 (FW->Sum)'),
    ]
    for pat, name in conn_checks:
        if not re.search(pat, sys43):
            print("[ERROR] 连线缺失: %s" % name)
            raise SystemExit(1)

    # 验证无旧连线
    if re.search(r'70#out:1[\s\S]*?52#in:1', sys43) and \
       not re.search(r'<Line>\s*<P Name="ZOrder">6</P>\s*<P Name="Src">70#out:1</P>\s*<P Name="Dst">52#in:1</P>\s*</Line>', sys43):
        # 旧连线 70->52 的残余不存在（70->52 只在 70->103 和 103->52 的跨行匹配中出现）
        # 我们只检查 70 和 52 在同一个 <Line> 中的情况
        lines = re.findall(r'<Line>.*?</Line>', sys43, re.DOTALL)
        for l in lines:
            if re.search(r'70#out:1', l) and re.search(r'52#in:1', l):
                print("[ERROR] 旧连线 70->52 未被移除")
                raise SystemExit(1)

    print("[OK]   XML 完整性检查通过")


if __name__ == '__main__':
    main()
