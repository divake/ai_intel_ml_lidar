"""Speaker-prep PDF — honest briefing to present + field questions. Output: results/CogniSense_brief.pdf"""
import os
from reportlab.lib.pagesizes import letter
from reportlab.lib.units import inch
from reportlab.lib.colors import HexColor
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.lib.enums import TA_LEFT
from reportlab.platypus import (SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle,
                                HRFlowable, ListFlowable, ListItem)
import sys; sys.path.insert(0, os.path.dirname(__file__))
import common as C

ACC = HexColor("#1f5fb4"); RED = HexColor("#c0392b"); INK = HexColor("#21252b"); MUT = HexColor("#5a6370")
out = os.path.join(C.RESULTS, "CogniSense_brief.pdf")
doc = SimpleDocTemplate(out, pagesize=letter, topMargin=0.7*inch, bottomMargin=0.7*inch,
                        leftMargin=0.8*inch, rightMargin=0.8*inch, title="CogniSense — speaker brief")
ss = getSampleStyleSheet()
def st(name, **kw):
    kw.setdefault("fontName", "Times-Roman"); kw.setdefault("textColor", INK)
    kw.setdefault("fontSize", 10.5); kw.setdefault("leading", 14)
    return ParagraphStyle(name, parent=ss["Normal"], **kw)
TITLE = st("t", fontName="Times-Bold", fontSize=19, leading=22, textColor=INK)
SUB = st("s", fontName="Times-Italic", fontSize=12, leading=15, textColor=MUT)
H1 = st("h1", fontName="Times-Bold", fontSize=14, leading=17, textColor=ACC, spaceBefore=12, spaceAfter=4)
BODY = st("b", fontSize=10.5, leading=14.5, alignment=TA_LEFT, spaceAfter=5)
Q = st("q", fontName="Times-Bold", fontSize=10.5, leading=14, textColor=INK, spaceBefore=6)
A = st("a", fontSize=10.5, leading=14, textColor=HexColor("#33373d"), spaceAfter=4, leftIndent=10)
SMALL = st("sm", fontSize=9.5, leading=12.5, textColor=MUT)

E = []
def P(t, s=BODY): E.append(Paragraph(t, s))
def bl(items, s=BODY):
    E.append(ListFlowable([ListItem(Paragraph(x, s), leftIndent=12, value="•") for x in items],
                          bulletType="bullet", start="•", leftIndent=14))
def rule(): E.append(HRFlowable(width="100%", thickness=0.8, color=HexColor("#d5dde6"), spaceBefore=4, spaceAfter=6))

# ---------- header ----------
P("Conformal Safety Bounds for LiDAR Localization", TITLE)
P("Speaker brief — what to say, the real numbers, and how to answer the hard questions.", SUB)
rule()

# ---------- pitch ----------
P("30-second pitch", H1)
P("We built a self-driving LiDAR robot that maps and drives a 250&nbsp;m building loop on its own. "
  "The interesting part is the question: <i>can it tell when it can no longer trust where it is?</i> "
  "Its on-board filter says it's confident to a few millimetres — but that confidence is wrong almost all of the time. "
  "We show that localization reliability is governed by how much the LiDAR can actually see, and we wrap a "
  "<b>conformal-prediction</b> safety bound around it: a distribution-free, online-valid estimate of the error "
  "that lets the robot slow down exactly when its sensing degrades — cutting unsafe events ~70% — where the "
  "over-confident filter would drive on blind.")

# ---------- the story ----------
P("The story (4 acts)", H1)
P("<b>Problem.</b> A LiDAR localizer outputs a single pose, no honest 'how sure'. In self-similar corridors the "
  "geometry is ambiguous and the estimate can drift or jump silently. The filter's own covariance looks tiny and "
  "trustworthy — but it isn't.")
P("<b>Demo.</b> Real robot (Agilex Scout-Mini), RoboSense Helios-16 LiDAR + RealSense IMU, FAST-LIO2 LiDAR-inertial "
  "SLAM, on an Intel NUC. It drove a 251.6&nbsp;m corridor loop autonomously and built an 801k-point loop-closed map. "
  "Two videos: the robot driving, and the live LiDAR map forming.")
P("<b>Solution.</b> Read the sensing quality the robot already has (scan density / effective range) → predict the "
  "localization error with conformal prediction (a distribution-free coverage guarantee, no Bayesian priors) → keep "
  "that guarantee valid online with Adaptive Conformal Inference (ACI) → govern speed on the bound.")
P("<b>Results.</b> (1) The filter's '90%' interval really covers ~1.3% (≈9× over-confident). (2) As effective range "
  "drops 40→3&nbsp;m, error grows ~4× and failures triple (8%→23%), with a directional 'aperture' (worse along the "
  "corridor). (3) A naive bound under-covers under shift (0.88); ACI restores 0.90 online. (4) A sensor-aware speed "
  "policy cuts unsafe events ~70% (3886→1148), slowing only as sensing degrades.")

# ---------- slide-by-slide ----------
P("Slide-by-slide talking points", H1)
sb = [
 ("1 · Title", "Set the frame: a self-driving LiDAR robot that knows when it can't trust where it is."),
 ("2 · Problem", "A localizer gives a pose, not an honest uncertainty; the filter's confidence is misleading. Point at the 250 m loop."),
 ("3 · Demo (videos)", "Play both: robot driving + live mapping. Say: real sensor, real building, on an edge box. This is the platform the rest sits on."),
 ("4 · Solution", "Walk the 4 boxes left→right. Emphasize: distribution-free, online, no Bayesian priors, edge-ready. The point is turning 'how sure' into a number AND an action."),
 ("5 · Over-confidence", "The headline number: its '90%' is right 1.3% of the time. Conclusion: don't gate safety on the filter."),
 ("6 · Degradation", "Reliability tracks sensing quality; the corridor aperture appears (along-error grows faster). This motivates a sensor-aware bound."),
 ("7 · Conformal", "Naive bound silently under-covers under shift; ACI fixes it online. This is the strongest, cleanest result — lean on it."),
 ("8 · Safe action (table)", "Walk the rows: full speed at 40 m, stops at 3–4 m; unsafe events 3886→1148. The filter can't do this — it never knows it's degraded."),
 ("9 · Takeaways", "Real robot + an uncertainty it can act on; the filter is overconfident; conformal gives the honest online bound; 70% fewer unsafe events; edge-ready."),
]
for h, t in sb:
    E.append(Paragraph(f"<b>{h}.</b> {t}", st("sbi", fontSize=10.5, leading=13.5, spaceAfter=3, leftIndent=4)))

# ---------- numbers ----------
P("Numbers you must know", H1)
tb = Table([
    ["Demo", "251.6 m loop · 801k-pt map · 7,619 scans · FAST-LIO2 (LiDAR+IMU) · Intel NUC"],
    ["Over-confidence", "EKF σ≈3.7 mm; its '90%' interval covers 1.3% of poses; ≈9× too confident"],
    ["Degradation", "range 40→3 m: error ~4×; failures 8%→23%; aperture ratio 1.06→1.77"],
    ["Conformal", "split under shift 0.88 → ACI 0.90 (target 0.90); fixed-anisotropic 8.4% tighter at matched coverage"],
    ["Safe action", "unsafe 3886→1148 (−70%); EKF-gate 3886→3309; slows 10%→100% as range drops"],
], colWidths=[1.35*inch, 5.55*inch])
tb.setStyle(TableStyle([("FONT", (0,0), (-1,-1), "Times-Roman", 9.5),
    ("FONT", (0,0), (0,-1), "Times-Bold", 9.5), ("TEXTCOLOR", (0,0), (0,-1), ACC),
    ("VALIGN", (0,0), (-1,-1), "TOP"), ("TOPPADDING",(0,0),(-1,-1),4), ("BOTTOMPADDING",(0,0),(-1,-1),4),
    ("LINEBELOW", (0,0), (-1,-2), 0.4, HexColor("#e0e6ee")), ("LEFTPADDING",(0,0),(-1,-1),6)]))
E.append(tb)

# ---------- Q&A ----------
P("Hard questions &amp; honest answers", H1)
P("Be straight on these — they are the obvious attacks. Honesty here is your strength.", SMALL)
qa = [
 ("Is this real or simulation?",
  "The robot, sensor, building, SLAM and the autonomous drive are 100% real. The <i>degradation sweep</i> is emulated by "
  "range-truncating the live scans (a stand-in for sparse / occluded / cheaper sensing) — say so plainly; it's a controlled "
  "stress test, and the trend is the claim, not the absolute failure rates."),
 ("Is the 9× over-confidence a fair comparison?",
  "Mostly. FAST-LIO's covariance is a local/per-step filter covariance; we compare it to a global re-localization error, so it's "
  "not perfectly apples-to-apples. But even read charitably it's 7–9× optimistic — the honest claim is 'it is not a safety bound', "
  "not 'the filter is broken'."),
 ("Does your observability signal actually predict failure?",
  "No — and we report that honestly. With a 360° LiDAR and a dense map, single-scan registration is robust, so a per-scan geometric "
  "observability score has ~no predictive power (AUROC≈0.5 within a sensing level). The thing that <i>does</i> predict reliability is "
  "the sensing quality (density/range) — which the robot observes directly. That negative result is part of the contribution."),
 ("Then is the 'conformal gate' really conformal?",
  "The gating <i>decision</i> triggers on observable sensing density — a plain density/range gate does about as well. Conformal's role "
  "is the <b>calibrated bound and the (held-out) false-alarm guarantee</b>, not the gating skill. Don't claim conformal beats the "
  "baseline at gating; claim it makes the bound trustworthy."),
 ("Is the anisotropic 8.4% win real?",
  "Yes, for a <i>fixed</i> anisotropic region at matched coverage. The per-scan <i>adaptive</i> version is only ~3.6% and slightly "
  "under-covers — so credit the fixed region, not the adaptive method."),
 ("Where's the ground truth?",
  "There is no survey GT. We use a pseudo-GT from injected-perturbation recovery (inject a known pose offset, recover by ICP, measure "
  "the residual). The absolute along-corridor drift is fundamentally unobservable to LiDAR — that's literally the aperture problem."),
 ("Coverage — marginal or conditional?",
  "Marginal. Per-degradation-level coverage ranges ~0.80–0.94 under the shift; ACI flattens it. Effective sample size ≈ number of scans "
  "(~3,800), not the 26k scan×level rows."),
 ("Why ACI / online conformal at all?",
  "The scan stream is non-exchangeable and shifts (open hall → junction → degraded). Plain split-conformal silently loses coverage; "
  "ACI (Gibbs–Candès) updates the level from realized miscoverage and restores long-run coverage — no retraining."),
 ("What's the actual novelty / contribution?",
  "The synthesis: a real autonomous LiDAR robot + a calibrated, online, sensor-aware localization-safety bound + the (clean) finding "
  "that the filter's confidence is unusable for safety. We also tested the elegant 'observability predicts error' idea and report it as "
  "an honest negative — which is itself useful."),
 ("Does it generalize?",
  "Single building, single run, one 16-beam sensor — this is a mechanism demonstration, not a population rate. Next steps: live "
  "closed-loop on the robot, richer degradation models, and a second environment."),
]
for q, a in qa:
    E.append(Paragraph("Q: " + q, Q)); E.append(Paragraph("A: " + a, A))

# ---------- glossary ----------
P("One-line glossary", H1)
bl([
 "<b>Conformal prediction</b> — a wrapper that turns any score into a prediction set/interval with a guaranteed coverage rate, distribution-free, finite-sample.",
 "<b>ACI (Adaptive Conformal Inference)</b> — online conformal: nudges the threshold from observed miscoverage so coverage holds even under distribution shift.",
 "<b>Coverage / PICP</b> — the fraction of times the true value falls inside the predicted interval (target here: 90%).",
 "<b>FAST-LIO2</b> — a tightly-coupled LiDAR-inertial odometry/SLAM system (the robot's localizer).",
 "<b>Aperture problem</b> — in a straight corridor the geometry doesn't constrain motion <i>along</i> the corridor, so that direction is poorly observable.",
], SMALL)

doc.build(E)
print("wrote", out, "(", round(os.path.getsize(out)/1024, 0), "KB )")
