# D10 — Global Fishing Watch anchorages: license gate, feasibility, and D3 impact

Date: 2026-07-08. Origin: data-expansion item D10
(`docs/superpowers/plans/2026-07-02-data-expansion-todos.md` §D10);
pre-water selection doc item 13. North-star tag: **Cl-3 corroboration seam**
(KEEP-side chart prior) + **D3 replacement**. Ticket:
`docs/superpowers/plans/2026-07-08-d10-gfw-anchorages-ticket.md`.

**Bottom line: NO-GO under the current license for navtracker's (TKMS)
commercial context.** Desk check only — **no registration, no download, no
data pulled**. The Step-0 license gate resolved to classification **(c) not
usable**, so per the ticket ("If (c), STOP and report") execution stopped
before Step 1. The roadmap-relevant assessments (Steps 2–3) are provided at
**desk-check level from public GFW documentation** (reading public web pages
is not use of the licensed data), each flagged moot-under-license. This
mirrors the D7 MOANA entry (2026-07-06): license is the decisive blocker, the
other dimensions are still assessed so the queue learns from it.

---

## 1. License classification (the gate) — **(c) NOT USABLE**

Global Fishing Watch ships **data products** under **CC BY-NC 4.0**
(NonCommercial). navtracker is a TKMS commercial product. NonCommercial bars
product-directed use — including *research-side statistics derived to inform
our product's design*, because that use is "directed towards commercial
advantage". There is no ShareAlike (lighter than D7 MOANA's CC-BY-NC-**SA**),
but the NonCommercial term **alone** is decisive, exactly as ruled for MOANA.

### Operative clauses (quoted verbatim)

**GFW Terms of Use** (`globalfishingwatch.org/terms-of-use/`), on data
products:

> "all of our publicly available images, screen shots, data products, or
> other content materials are provided for and subject to non-commercial use
> in accordance with the Creative Commons Attribution Non-Commercial 4.0
> license"

> "The Site and the Services are provided for Non-Commercial use only in
> accordance with the CC BY-NC 4.0 license. If you would like to use the Site
> and/or the Services for commercial purposes, please contact us."

Required attribution format, if ever used non-commercially:

> "Copyright [year], Global Fishing Watch, Inc. Accessed on [date]. [Link to
> the page]."

**CC BY-NC 4.0 legal code** (`creativecommons.org/licenses/by-nc/4.0/`):

> "NonCommercial means not primarily intended for or directed towards
> commercial advantage or monetary compensation."

> rights are granted to "reproduce and Share the Licensed Material, in whole
> or in part, for NonCommercial purposes only"

**GFW API documentation** (`globalfishingwatch.org/our-apis/documentation`),
covering the Events API (anchorage visits, port calls) named in D10:

> "Global Fishing Watch APIs are only available for non-commercial purposes.
> They are used by researchers, governments and technology companies."

API access additionally requires: (1) register a GFW account, (2) create an
API access token, (3) **agree to the non-commercial terms and attribute GFW**.

### The Apache-2.0 red herring (checked and ruled out)

The ToU also says *"Many of the products available for download from this site
are licensed under the Apache 2.0 license"*, and the GFW anchorage **pipeline
repo** (`github.com/GlobalFishingWatch/anchorages_pipeline`) carries an
Apache-2.0 `LICENSE`. That Apache grant covers the **software/code** (the
pipeline you could run yourself), **not the data product**. The repo has no
separate data-license statement; the controlling license for the *anchorage
database and event data* is the ToU's CC BY-NC 4.0. So Apache 2.0 does **not**
open a commercial path to the data. (If we ever ran the open-source pipeline on
a *commercially-clean AIS source of our own*, that output would be ours — see
§5, MarineCadastre path — but that is D3 by another name, not GFW data.)

### Why not classification (b) "research-side only"

The ticket left (b) — "usable only as research-side statistics that inform our
own design (numbers derived by us, data never shipped)" — open. It does not
survive the NonCommercial definition: statistics derived **to inform a
commercial product's design** are "directed towards commercial advantage".
(b) would be lawful only for genuinely non-commercial research (e.g. an
academic publication), which this is not. So for *our* purpose the honest
verdict collapses to (c).

**Lawful paths that remain:** (i) a commercial-use grant negotiated with GFW
("please contact us") — an organizational decision for the user, out of scope
here; or (ii) the commercially-clean substitute of §5 (MarineCadastre /
official ENC charts).

**Registration wall (independent blocker).** Even setting the data license
aside, the API and bulk download require creating an account and *agreeing to
the non-commercial terms* — which a commercial entity cannot truthfully accept
for product use. This independently trips the ticket's stop-and-report
condition on registration.

---

## 2. Data extracts — **none** (Step 1 not executed)

No GFW data was registered for, downloaded, or written to `data/gfw/`. There
is therefore nothing to checksum and nothing gitignored to record. Step 1 was
gated on a passing license, which it did not. This section exists to state the
absence explicitly (acceptance item 2).

---

## 3. Boston sanity anchor (Step 2) — **not measured; reasoned expectation = likely MISS**

Step 2 asks whether our one owned ground-truth point — the video-verified KEEP
cluster at **≈42.3585 N, −71.0464** (the "largest single driver" of
`KEEP_INCOV_UNCHARTED`, `evaluation-log.md`; uncharted, in-coverage,
radar-supported, small recreational craft in the Boston inner-harbour / Charles
basin area) — appears in the GFW anchorage layer.

**This was not measured** (no download; license stop). On paper the expected
result is a **MISS**, and the *reason* is the informative part:

- GFW's anchorage database is built from **AIS**: locations where **≥20 unique
  AIS-transmitting vessels** sat stationary since 2012.
- Our cluster is **small recreational craft** (moored yachts/sailboats). Class-B
  AIS is optional on such craft and frequently absent; the cluster is
  "chart-silent" precisely because it is informal recreational moorings, not a
  designated commercial anchorage.
- So the very targets that generate our **hardest KEEP cases** are the ones GFW
  is structurally blind to. Per the ticket, a miss here is "a finding about
  GFW's coverage threshold, not a failure" — and it is a *load-bearing* finding
  for §4's prior assessment: a GFW-anchorage KEEP prior would light up
  commercial anchorages while staying dark over exactly the AIS-dark
  small-craft fields where we most need help.

(Boston *does* host large designated commercial anchorages — President Roads,
Broad Sound — well north/east of our cluster; GFW would very plausibly list
those. That does not help the specific cluster we own.)

---

## 4. The two assessments (Step 3)

### 4a. D3 replacement — per-statistic coverage

D3 wanted anchored-vessel **dwell / transition / watch-circle** statistics for
the AIS-veto and anchored-vessel corroboration design. What GFW *structurally*
offers (from public docs — resolution, product shape), and whether it would
shrink D3 **if the license allowed** (it does not):

| D3 statistic | In GFW (on paper) | Verdict | Note |
|---|---|---|---|
| Anchorage **locations** (where anchoring happens) | Anchorage DB = named global list of ≥20-vessel stationary clusters (S2-cell aggregation, ~0.5 km) | **Covered** | Coarse (~500 m cells), AIS-only; commercial anchorages only (see §3) |
| **Dwell-time** distribution (how long vessels sit) | Events API anchorage-visit / port-visit events carry entry+exit timestamps | **Derivable** (we aggregate; not pre-computed as a distribution) | Per-event, per-vessel; would need our own binning |
| **Transition** stats (anchored↔underway rate/timing) | Visit start/end + voyage (port-call) sequences | **Partially derivable** | The underlying "is anchored" is GFW's AIS speed-threshold — the *same AIS signal the veto already has*; adds little the veto lacks |
| **Watch-circle** radius (per-vessel swing at anchor) | — | **Not covered** | GFW aggregates to ~500 m S2 cells; watch-circle is a tens-of-metres per-vessel kinematic quantity. Needs raw per-vessel AIS position time series |

**Does D10 shorten D3? No — for two independent reasons.**

1. **License.** Every "covered/derivable" cell above is CC-BY-NC and unusable
   for our product. GFW cannot legally shrink D3 for us.
2. **Substance.** Even ignoring the license, the one statistic the veto most
   needs — the **watch-circle swing radius** (to size the anchored-vessel gate)
   — is *not* in GFW at any resolution, and the dwell/transition signals are
   AIS-derived, i.e. the veto's *existing* data path rather than new
   information. GFW pre-computes the parts D3 needed least and omits the part it
   needed most.

**Net for D3:** D10 does **not** replace D3. D3's original **MarineCadastre**
route (NOAA/BOEM AIS, **U.S. public domain — commercially clean**) remains the
lawful and substantively necessary path for watch-circle and per-vessel dwell
mining. D3 is *not* shortened; if anything this sharpens D3's scope to "raw
per-vessel AIS at anchor → watch-circle + dwell", which GFW cannot supply.

### 4b. KEEP-side corroboration prior — assessment note (no wiring)

Idea: anchorage polygons as a chart prior "this water is a known anchorage →
bias KEEP" — opposite polarity to the charted-structure SUPPRESS prior, feeding
the same corroboration seam.

- **Polygon quality/size.** GFW anchorages are **S2-cell centroids/clusters
  (~0.5 km)**, not tight surveyed polygons. As a KEEP region you'd get a coarse
  ~500 m blob around a centroid — imprecise at harbour scale.
- **False-positive risk (do they cover fairways?).** **Low by construction** —
  anchorages are where ≥20 vessels *stop*; fairways are transited, not stopped
  in, so they should not qualify. This is a genuine structural plus. Residual
  risk: at ~500 m resolution a coastal anchorage cell can bleed into an adjacent
  channel, and dual-use "anchorage + waiting area near a fairway" spots exist.
- **Coverage blind spot (the §3 finding).** The prior would be **dark over
  AIS-dark small-craft mooring fields** — exactly the cluster we own and the
  hardest KEEP cases. It helps least where we need it most.
- **Wiring (note only, not implemented).** A polygon set where a plot/track
  falling inside a known-anchorage polygon raises KEEP bias / lowers SUPPRESS,
  entering the same corroboration seam as chart/camera/AIS/coop. Per project
  rule this would be a per-instance ctor-threaded prior (no global toggle),
  gated on synthetic truth, presence-as-KEEP-evidence-only (absence never
  suppresses) — the same discipline as the camera-bearing channel.

**Recommendation for the prior:** the *right* source for a "known anchorage →
KEEP" chart prior is the **official ENC / S-57 charted anchorage-area feature
(object class `ACHARE`, and anchor berths `ACHBRT`)** — an authoritative,
surveyed **polygon**, already in the chart-corroboration substrate the project
uses, with none of GFW's coarse-resolution, AIS-blind-spot, or license
problems. GFW would at best be a non-commercial cross-check on ACHARE coverage,
and it is license-barred for us regardless. **Do not wire a GFW-derived KEEP
prior; wire the chart's ACHARE polygons when the corroboration seam lands.**

---

## 5. Net and recommendation

- **License = (c) NOT USABLE** for TKMS's commercial context, data and API
  alike (CC BY-NC 4.0 + non-commercial API terms). Same NonCommercial ground
  that made D7 MOANA a NO-GO; GFW is lighter (no ShareAlike) but still barred.
- **D3 is not replaced or shortened** — GFW omits the watch-circle statistic
  D3 most needs, and its dwell/transition signals are AIS-derived (the veto's
  existing path). **MarineCadastre** (U.S. public-domain AIS) stays the lawful,
  necessary D3 route.
- **The KEEP-prior should be built from charted ENC `ACHARE` anchorage-area
  polygons**, not GFW clusters — authoritative, surveyed, commercially clean,
  and already in the chart substrate.
- **If GFW is ever wanted anyway**, the only lawful path is a commercial-use
  grant from GFW ("please contact us") — an organizational decision for the
  user, not an extraction task.

**What to test next (unblocked by this):** when the chart-corroboration seam is
picked up, prototype the KEEP prior against ENC `ACHARE` polygons at the philos
and HAXR geographies on **synthetic truth**, and confirm the §4b false-positive
(fairway-bleed) and coverage-blind-spot (small-craft) properties on real charts.
D3's watch-circle mining stays queued against MarineCadastre.

---

## Sources

- GFW Terms of Use — `globalfishingwatch.org/terms-of-use/`
- GFW Anchorages, Ports and Voyages —
  `globalfishingwatch.org/datasets-and-code-anchorages/`
- GFW API documentation — `globalfishingwatch.org/our-apis/documentation`
- GFW anchorage pipeline (code, Apache-2.0) —
  `github.com/GlobalFishingWatch/anchorages_pipeline`
- CC BY-NC 4.0 legal code — `creativecommons.org/licenses/by-nc/4.0/legalcode`
- D7 MOANA precedent — `docs/algorithms/evaluation-log.md` (2026-07-06 entry)
- Our video-verified cluster — `docs/algorithms/evaluation-log.md`
  (`KEEP_INCOV_UNCHARTED`, 42.3585 N / −71.0464)
</content>
</invoke>
