# control/report_builder.py
#
# Builds a text report for the session file end summary.
# Keep it simple, append-only.

def build_summary(*,
                  settings: dict,
                  final_state: str,
                  fault: str,
                  r_charge_mohm: int,
                  r_discharge_mohm: int,
                  discharged_ah: float,
                  notes: str = None) -> str:
    lines = []
    lines.append("# SUMMARY")
    lines.append("final_state={}".format(final_state))
    lines.append("fault={}".format(fault if fault else "NONE"))
    lines.append("r_charge_mohm={}".format(r_charge_mohm))
    lines.append("r_discharge_mohm={}".format(r_discharge_mohm))
    lines.append("discharged_ah={:.4f}".format(float(discharged_ah)))

    # Settings snapshot
    lines.append("# SETTINGS")
    for k in sorted(settings.keys()):
        lines.append("{}={}".format(k, settings[k]))

    if notes:
        lines.append("# NOTES")
        lines.append(str(notes))

    lines.append("")  # trailing newline
    return "\n".join(lines)
