import React, { useState } from "react";
import { ReportSection, RichNarrative, DataComponent, useDataApp } from "../../data-app-public.jsx";
import content from "./season-content.json";
import img0 from "../assets/electrical.png";
import img1 from "../assets/swerve.png";
import img2 from "../assets/mechanisms.png";
import img3 from "../assets/motors.png";
import img4 from "../assets/comparison.png";
import img5 from "../assets/video-q11.jpg";
import img6 from "../assets/video-recovery.jpg";
import img7 from "../assets/github-auto.png";
import img8 from "../assets/github-steering.png";
const images={"electrical":img0,"swerve":img1,"mechanisms":img2,"motors":img3,"comparison":img4,"video-q11":img5,"video-recovery":img6,"github-auto":img7,"github-steering":img8};
export function ReportContent(){
 const {snapshot,visible,appTitle}=useDataApp(); const [event,setEvent]=useState('All events');
 const all=snapshot.queries.matches.rows; const rows=all.filter(r=>event==='All events'||r.event===event);
 return <article className="report-content season-report" aria-label="Team 1257 technical season report">
 <header className="report-hero"><RichNarrative id="season:eyebrow" value="FRC 1257 / 2026 TECHNICAL RETROSPECTIVE"/><h1>{appTitle}</h1><RichNarrative id="season:deck" value="A season told through robot telemetry, code history and broadcast evidence. January–March competition archive, with later development through May. Prepared October 16, 2026."/></header>
 <nav className="season-toc" aria-label="Report contents">{content.sections.map(s=><a key={s.id} href={'#chapter-'+s.id}>{s.title}</a>)}</nav>
 {content.sections.map(s=>visible(s.id)&&<section id={'chapter-'+s.id} key={s.id} className="season-chapter">
 <ReportSection id={s.id} title={s.title} queryId={s.query} sourceRows={snapshot.queries[s.query].rows} showHeading={false}><RichNarrative id={s.id+':body'} value={s.text}/></ReportSection>
 {s.id==='competitions'&&<DataComponent id="match-results" title="All 29 recorded match appearances" queryId="matches" sourceRows={all} displayRows={rows} kind="table"><label className="season-filter">Match ledger event <select value={event} onChange={e=>setEvent(e.target.value)}><option>All events</option><option>Tabernacle</option><option>Bensalem</option></select></label><div className="season-table"><table data-reviewed-rows><thead><tr>{['Event / match','Stage','1257 alliance','Opponent','Result','Telemetry'].map(x=><th key={x}>{x}</th>)}</tr></thead><tbody>{rows.map(r=><tr key={r.match}><td><a href={'https://www.thebluealliance.com/match/'+r.match} target="_blank" rel="noreferrer">{r.event} {r.match.split('_')[1].toUpperCase()}</a></td><td>{r.stage}</td><td className="num">{r.alliance_score}</td><td className="num">{r.opponent_score}</td><td>{r.result}</td><td>{r.telemetry_coverage}</td></tr>)}</tbody></table></div></DataComponent>}
 {s.figures.map((f,i)=><DataComponent id={s.id+'-figure-'+i} key={f.asset} title={f.asset.startsWith('github')?'Source code evidence':f.asset.startsWith('video')?'Broadcast evidence':'Telemetry evidence'} queryId={s.query} sourceRows={snapshot.queries[s.query].rows} kind="custom"><figure className="season-figure"><a href={images[f.asset]} target="_blank" rel="noreferrer" aria-label={'Open full-size '+f.asset}><img src={images[f.asset]} alt={f.caption.replace(/\[|\]/g,'')} loading="lazy"/></a><figcaption><RichNarrative id={s.id+':caption:'+i} value={f.caption}/></figcaption></figure></DataComponent>)}
 </section>)}
 </article>
}
