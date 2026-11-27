#!/usr/bin/env python3
"""Daily visitor counts of the online demo, from the nginx access log.

    planelands-visits            print the table (and update the record)
    planelands-visits --quiet    update the record only (cron)
    planelands-visits --days 60  how many days to print (default 30)

Reads /var/log/nginx/planelands.access.log and its rotated copies (.1, .2.gz ...; nginx keeps 14 days)
and writes one line per day (Beijing time) to /var/log/planelands/visits.tsv:

    date  visitors  ips  opens  countries

    opens     page loads: GET of the page itself (/ or /index.html, any query) answered 200 / 304;
              the release files, data, Pyodide and the version check every 5 minutes do not count
    visitors  distinct (IP, browser) pairs among those page loads
    ips       distinct IPs
    countries the most frequent visitor countries (Cloudflare CF-IPCountry)

Requests from crawlers, scripts and headless browsers are left out.  Days that have dropped out of the
rotated logs keep the line they had; the oldest day still in the logs may have lost its first hours
to rotation and does not overwrite a line written earlier.  Installed by README “在线演示的部署”.
"""
from __future__ import annotations

import argparse
import collections
import datetime as dt
import glob
import gzip
import os
import re
import sys

LOG = "/var/log/nginx/planelands.access.log"
OUT = "/var/log/planelands/visits.tsv"
TZ = dt.timezone(dt.timedelta(hours=8), "CST")       # day boundaries in Beijing time

# planelands format: ip [time] "req" status bytes "ref" "ua" "country"; also the older combined format
LINE = re.compile(r'^(\S+) (?:\S+ \S+ )?\[([^\]]+)\] "([^"]*)" (\d{3}) \S+ "[^"]*" "([^"]*)"(?: "([^"]*)")?')
PAGE = re.compile(r'^GET (/|/index\.html)(\?\S*)? HTTP/')
BOT = re.compile(r'bot|spider|crawl|slurp|curl|wget|python|go-http|httpclient|java/|okhttp|headless|'
                 r'monitor|uptime|scanner|preview|facebookexternalhit|lighthouse|^-?$', re.I)


def log_files():
    """the current log and its rotated copies, oldest first"""
    rot = re.compile(r"\.(\d+)(\.gz)?$")
    files = [f for f in glob.glob(LOG + ".*") if rot.search(f)] + [LOG]
    rank = lambda f: int(rot.search(f).group(1)) if f != LOG else 0
    return sorted((f for f in files if os.path.isfile(f)), key=rank, reverse=True)


def read_lines(path):
    op = gzip.open if path.endswith(".gz") else open
    with op(path, "rt", encoding="utf-8", errors="replace") as f:
        yield from f


def count():
    days = collections.defaultdict(lambda: {"opens": 0, "visitors": set(), "ips": set(), "cc": collections.Counter()})
    first = None
    for path in log_files():
        for line in read_lines(path):
            m = LINE.match(line)
            if not m:
                continue
            ip, tl, req, status, ua, cc = m.groups()
            try:
                t = dt.datetime.strptime(tl, "%d/%b/%Y:%H:%M:%S %z").astimezone(TZ)
            except ValueError:
                continue
            day = t.date().isoformat()
            first = day if first is None or day < first else first
            if status not in ("200", "304") or not PAGE.match(req) or BOT.search(ua):
                continue
            d = days[day]
            d["opens"] += 1
            d["visitors"].add((ip, ua))
            d["ips"].add(ip)
            if cc and cc not in ("-", "XX", "T1"):
                d["cc"][cc] += 1
    return days, first


def load(path):
    rows = {}
    if os.path.isfile(path):
        with open(path, encoding="utf-8") as f:
            for line in f:
                if line.startswith("#") or not line.strip():
                    continue
                p = line.rstrip("\n").split("\t")
                rows[p[0]] = p
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--quiet", action="store_true", help="update the record only")
    ap.add_argument("--days", type=int, default=30, help="days to print (default 30)")
    ap.add_argument("--out", default=OUT)
    a = ap.parse_args()
    days, first = count()
    rows = load(a.out)
    for day, d in days.items():
        if day == first and day in rows:
            continue                                   # partly rotated away: keep the earlier line
        cc = ",".join(f"{c}:{n}" for c, n in d["cc"].most_common(5))
        rows[day] = [day, str(len(d["visitors"])), str(len(d["ips"])), str(d["opens"]), cc]
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    tmp = a.out + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        f.write("# date\tvisitors\tips\topens\tcountries   (Beijing time; see planelands-visits --help)\n")
        for day in sorted(rows):
            f.write("\t".join(rows[day]) + "\n")
    os.replace(tmp, a.out)
    if a.quiet:
        return
    print(f"{'日期':<10}  {'访客':>6}  {'IP':>6}  {'打开次数':>6}  国家/地区")
    for day in sorted(rows)[-a.days:]:
        r = rows[day] + [""] * 5
        print(f"{r[0]:<10}  {r[1]:>6}  {r[2]:>6}  {r[3]:>8}  {r[4]}")
    print(f"(记录：{a.out}；北京时间按天，访客 = 不同的 IP + 浏览器)")


if __name__ == "__main__":
    sys.exit(main())
