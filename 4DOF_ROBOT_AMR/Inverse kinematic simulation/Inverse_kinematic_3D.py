"""
  4-DOF Articulated Robot Arm — 3D Kinematics Simulator
  J1: Base rotation (Z), J2: Shoulder, J3: Elbow, J4: Wrist
  Base tại gốc tọa độ (0,0,0) trên mặt phẳng XY
"""
import numpy as np, tkinter as tk, math
from tkinter import ttk, font as tkfont
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

LINK_LENGTHS = [90, 90, 30]
BASE_HEIGHT = 75
JOINT_LIMITS = [(-180,180),(-90,135),(-135,135),(-180,180)]
BG_DARK="#0d0d1a"; BG_PANEL="#141428"; BG_CANVAS="#0a0a1e"
ACCENT="#e94560"; ACCENT2="#0ea5e9"; ACCENT3="#22d3ee"; ACCENT4="#a78bfa"
TEXT_LIGHT="#e2e8f0"; TEXT_DIM="#94a3b8"; GRID_COLOR="#1e2a4a"
TARGET_COLOR="#facc15"; SUCCESS_CLR="#4ade80"; WARN_CLR="#fb923c"
JOINT_COLORS=["#e94560","#0ea5e9","#22d3ee","#a78bfa"]
LINK_COLORS=["#f87171","#38bdf8","#67e8f9"]

def dh_matrix(theta,d,a,alpha):
    ct,st=math.cos(theta),math.sin(theta)
    ca,sa=math.cos(alpha),math.sin(alpha)
    return np.array([[ct,-st*ca,st*sa,a*ct],[st,ct*ca,-ct*sa,a*st],[0,sa,ca,d],[0,0,0,1]])

def forward_kinematics_3d(thetas_deg, lengths=LINK_LENGTHS):
    """FK: 4 DH rows. Returns 4 points [Base, EndL1, EndL2, EE]."""
    L1,L2,L3=lengths
    th=[math.radians(t) for t in thetas_deg]
    dh=[(th[0],BASE_HEIGHT,0,math.pi/2),(th[1],0,L1,0),(th[2],0,L2,0),(th[3],0,L3,0)]
    T=np.eye(4); positions=[(0.,0.,BASE_HEIGHT)]
    for i,(theta,d,a,alpha) in enumerate(dh):
        T=T@dh_matrix(theta,d,a,alpha)
        if i==0: continue
        p=T[:3,3]; positions.append((p[0],p[1],p[2]))
    return positions

def jacobian_3d(thetas_deg, lengths=LINK_LENGTHS, delta=0.01):
    n=len(thetas_deg); J=np.zeros((3,n))
    ee0=np.array(forward_kinematics_3d(thetas_deg,lengths)[-1])
    for i in range(n):
        tp=list(thetas_deg); tp[i]+=delta
        ee1=np.array(forward_kinematics_3d(tp,lengths)[-1])
        J[:,i]=(ee1-ee0)/math.radians(delta)
    return J

def inverse_kinematics_3d(target_xyz, thetas_init_deg, lengths=LINK_LENGTHS,
                           max_iter=1000, tol=0.005, alpha=0.4, clamp=4.0):
    thetas=np.array(thetas_init_deg,dtype=float); target=np.array(target_xyz,dtype=float)
    err_norm=999.0
    for _ in range(max_iter):
        ee=np.array(forward_kinematics_3d(thetas,lengths)[-1])
        error=target-ee; err_norm=np.linalg.norm(error)
        if err_norm<tol: break
        J=jacobian_3d(thetas,lengths); lam=0.8; JT=J.T
        dtheta=JT@np.linalg.solve(J@JT+lam**2*np.eye(3),error)
        dtheta_deg=np.degrees(dtheta)*alpha
        dtheta_deg=np.clip(dtheta_deg,-clamp,clamp); thetas+=dtheta_deg
        for i in range(len(thetas)):
            thetas[i]=np.clip(thetas[i],JOINT_LIMITS[i][0],JOINT_LIMITS[i][1])
        thetas[0]=(thetas[0]+180)%360-180
    return thetas.tolist(), err_norm

class RobotArm3DSimulator:
    NJ=4; NL=3
    def __init__(self, root):
        self.root=root; root.title("4-DOF Robot Arm — 3D Simulator")
        root.configure(bg=BG_DARK); root.state("zoomed"); root.minsize(1280,750)
        self.thetas=[0.0,60.0,-30.0,0.0]; self.target=None; self.ik_error=0.0
        self.anim_id=None; self.view_elev=25; self.view_azim=-60; self.trail_points=[]
        self.font_title=tkfont.Font(family="Segoe UI",size=15,weight="bold")
        self.font_label=tkfont.Font(family="Segoe UI",size=11)
        self.font_value=tkfont.Font(family="Consolas",size=12,weight="bold")
        self.font_info=tkfont.Font(family="Consolas",size=10)
        self.font_small=tkfont.Font(family="Segoe UI",size=9)
        self.font_btn=tkfont.Font(family="Segoe UI",size=10,weight="bold")
        self._build_ui(); self._draw()

    def _build_ui(self):
        tf=tk.Frame(self.root,bg=BG_DARK,pady=6); tf.pack(fill="x")
        tk.Label(tf,text="🤖  4-DOF Robot Arm  —  3D Kinematics Simulator",
                 font=self.font_title,bg=BG_DARK,fg=TEXT_LIGHT).pack()
        tk.Label(tf,text="J1 xoay quanh Z | J2 Shoulder | J3 Elbow | J4 Wrist | Kéo canvas xoay góc nhìn",
                 font=self.font_small,bg=BG_DARK,fg=TEXT_DIM).pack()
        main=tk.Frame(self.root,bg=BG_DARK); main.pack(fill="both",expand=True,padx=8,pady=(0,8))
        co=tk.Frame(main,bg=ACCENT,bd=2); co.pack(side="left",fill="both",expand=True,padx=(0,8))
        self.fig=Figure(facecolor=BG_CANVAS,dpi=100)
        self.ax=self.fig.add_subplot(111,projection='3d')
        self.canvas=FigureCanvasTkAgg(self.fig,master=co)
        self.canvas.get_tk_widget().pack(fill="both",expand=True)
        rc=tk.Frame(main,bg=BG_PANEL,width=390,bd=0); rc.pack(side="right",fill="y"); rc.pack_propagate(False)
        self.pc=tk.Canvas(rc,bg=BG_PANEL,highlightthickness=0,bd=0)
        sb=tk.Scrollbar(rc,orient="vertical",command=self.pc.yview)
        self.pc.configure(yscrollcommand=sb.set); sb.pack(side="right",fill="y")
        self.pc.pack(side="left",fill="both",expand=True)
        panel=tk.Frame(self.pc,bg=BG_PANEL,padx=14,pady=10)
        self.pw=self.pc.create_window((0,0),window=panel,anchor="nw")
        panel.bind("<Configure>",lambda e:self.pc.configure(scrollregion=self.pc.bbox("all")))
        self.pc.bind("<Configure>",lambda e:self.pc.itemconfig(self.pw,width=e.width))
        self.pc.bind_all("<MouseWheel>",lambda e:self.pc.yview_scroll(int(-1*(e.delta/120)),"units"))
        # Sliders
        tk.Label(panel,text="🔧  Joint Angles",font=self.font_title,bg=BG_PANEL,fg=TEXT_LIGHT).pack(anchor="w",pady=(0,6))
        self.sliders=[]; self.angle_labels=[]
        jnames=["θ₁ (Base Rotation ↻Z)","θ₂ (Shoulder Pitch)","θ₃ (Elbow Pitch)","θ₄ (Wrist Pitch)"]
        link_info=["rotation","L1=120mm","L2=80mm","L3=50mm"]
        for i in range(self.NJ):
            f=tk.Frame(panel,bg=BG_PANEL); f.pack(fill="x",pady=3)
            hdr=tk.Frame(f,bg=BG_PANEL); hdr.pack(fill="x")
            dot=tk.Canvas(hdr,width=12,height=12,bg=BG_PANEL,highlightthickness=0)
            dot.pack(side="left",padx=(0,5)); dot.create_oval(2,2,12,12,fill=JOINT_COLORS[i],outline="")
            tk.Label(hdr,text=jnames[i],font=self.font_label,bg=BG_PANEL,fg=TEXT_LIGHT).pack(side="left")
            vl=tk.Label(hdr,text=f"{self.thetas[i]:+.1f}°",font=self.font_value,bg=BG_PANEL,fg=JOINT_COLORS[i])
            vl.pack(side="right"); self.angle_labels.append(vl)
            lo,hi=JOINT_LIMITS[i]
            sl=tk.Scale(f,from_=lo,to=hi,orient="horizontal",resolution=0.5,length=320,sliderlength=18,
                        bg=BG_PANEL,fg=TEXT_LIGHT,troughcolor=BG_DARK,activebackground=JOINT_COLORS[i],
                        highlightthickness=0,bd=0,showvalue=False,command=lambda v,idx=i:self._on_slider(idx,v))
            sl.set(self.thetas[i]); sl.pack(fill="x",pady=(1,0)); self.sliders.append(sl)
            mm=tk.Frame(f,bg=BG_PANEL); mm.pack(fill="x")
            tk.Label(mm,text=f"{lo}°",font=self.font_small,bg=BG_PANEL,fg=TEXT_DIM).pack(side="left")
            tk.Label(mm,text=link_info[i],font=self.font_small,bg=BG_PANEL,fg=TEXT_DIM).pack(side="left",expand=True)
            tk.Label(mm,text=f"{hi}°",font=self.font_small,bg=BG_PANEL,fg=TEXT_DIM).pack(side="right")
        ttk.Separator(panel,orient="horizontal").pack(fill="x",pady=10)
        # EE info
        tk.Label(panel,text="📍  End-Effector",font=self.font_title,bg=BG_PANEL,fg=TEXT_LIGHT).pack(anchor="w",pady=(0,5))
        inf=tk.Frame(panel,bg=BG_DARK,bd=1,relief="solid",padx=12,pady=8); inf.pack(fill="x")
        self.lbl_ee_x=tk.Label(inf,text="X: ---",font=self.font_info,bg=BG_DARK,fg=ACCENT,anchor="w"); self.lbl_ee_x.pack(fill="x")
        self.lbl_ee_y=tk.Label(inf,text="Y: ---",font=self.font_info,bg=BG_DARK,fg=ACCENT2,anchor="w"); self.lbl_ee_y.pack(fill="x")
        self.lbl_ee_z=tk.Label(inf,text="Z: ---",font=self.font_info,bg=BG_DARK,fg=ACCENT3,anchor="w"); self.lbl_ee_z.pack(fill="x")
        self.lbl_ee_d=tk.Label(inf,text="Dist: ---",font=self.font_info,bg=BG_DARK,fg=TEXT_DIM,anchor="w"); self.lbl_ee_d.pack(fill="x")
        ttk.Separator(panel,orient="horizontal").pack(fill="x",pady=10)
        # Line Target input
        tk.Label(panel,text="🎯  Straight Line Target (X, Y, Z)",font=self.font_title,bg=BG_PANEL,fg=TEXT_LIGHT).pack(anchor="w",pady=(0,5))
        lif=tk.Frame(panel,bg=BG_PANEL); lif.pack(fill="x",pady=(0,6))
        self.line_entries={}
        for lt,c in [("X",ACCENT),("Y",ACCENT2),("Z",ACCENT3)]:
            ef=tk.Frame(lif,bg=BG_PANEL); ef.pack(side="left",expand=True,fill="x",padx=1)
            tk.Label(ef,text=lt,font=self.font_label,bg=BG_PANEL,fg=c).pack(side="left")
            en=tk.Entry(ef,width=5,font=self.font_info,bg=BG_DARK,fg=TEXT_LIGHT,insertbackground=TEXT_LIGHT,bd=1,relief="solid")
            en.pack(side="left",padx=(2,0)); self.line_entries[lt]=en
        tk.Button(panel,text="▶  Move (Straight Line)",font=self.font_btn,bg=ACCENT,fg="white",activebackground="#be123c",
                  bd=0,padx=14,pady=5,cursor="hand2",command=self._solve_line_ik).pack(fill="x",pady=(0,6))

        ttk.Separator(panel,orient="horizontal").pack(fill="x",pady=10)
        # Circle Target input
        tk.Label(panel,text="🎯  Circle Trajectory (Xc, Yc, Zc, R)",font=self.font_title,bg=BG_PANEL,fg=TEXT_LIGHT).pack(anchor="w",pady=(0,5))
        tif=tk.Frame(panel,bg=BG_PANEL); tif.pack(fill="x",pady=(0,6))
        self.target_entries={}
        for lt,c in [("Xc",ACCENT),("Yc",ACCENT2),("Zc",ACCENT3),("R",ACCENT4)]:
            ef=tk.Frame(tif,bg=BG_PANEL); ef.pack(side="left",expand=True,fill="x",padx=1)
            tk.Label(ef,text=lt,font=self.font_label,bg=BG_PANEL,fg=c).pack(side="left")
            en=tk.Entry(ef,width=5,font=self.font_info,bg=BG_DARK,fg=TEXT_LIGHT,insertbackground=TEXT_LIGHT,bd=1,relief="solid")
            en.pack(side="left",padx=(2,0)); self.target_entries[lt]=en
        tk.Button(panel,text="▶  Run Trajectory",font=self.font_btn,bg=ACCENT2,fg="white",activebackground="#0284c7",
                  bd=0,padx=14,pady=5,cursor="hand2",command=self._solve_ik).pack(fill="x",pady=(0,6))
        tsf=tk.Frame(panel,bg=BG_DARK,bd=1,relief="solid",padx=12,pady=6); tsf.pack(fill="x")
        self.lbl_tgt=tk.Label(tsf,text="No target set",font=self.font_info,bg=BG_DARK,fg=TARGET_COLOR); self.lbl_tgt.pack(fill="x")
        self.lbl_ik_err=tk.Label(tsf,text="",font=self.font_info,bg=BG_DARK,fg=ACCENT); self.lbl_ik_err.pack(fill="x")
        self.lbl_ik_st=tk.Label(tsf,text="",font=self.font_info,bg=BG_DARK,fg=SUCCESS_CLR); self.lbl_ik_st.pack(fill="x")
        # Buttons
        bf=tk.Frame(panel,bg=BG_PANEL); bf.pack(fill="x",pady=(10,0))
        tk.Button(bf,text="↺ Reset",font=self.font_btn,bg=ACCENT,fg="white",bd=0,padx=10,pady=5,cursor="hand2",
                  command=self._reset).pack(side="left",expand=True,fill="x",padx=(0,3))
        tk.Button(bf,text="✕ Clear Target",font=self.font_btn,bg="#334155",fg=TEXT_LIGHT,bd=0,padx=10,pady=5,cursor="hand2",
                  command=self._clear_target).pack(side="left",expand=True,fill="x",padx=(3,3))
        tk.Button(bf,text="⟲ Clear Trail",font=self.font_btn,bg="#334155",fg=TEXT_LIGHT,bd=0,padx=10,pady=5,cursor="hand2",
                  command=self._clear_trail).pack(side="left",expand=True,fill="x",padx=(3,0))
        # Camera
        ttk.Separator(panel,orient="horizontal").pack(fill="x",pady=10)
        tk.Label(panel,text="🎥  Camera",font=self.font_label,bg=BG_PANEL,fg=TEXT_LIGHT).pack(anchor="w",pady=(0,5))
        vf=tk.Frame(panel,bg=BG_PANEL); vf.pack(fill="x")
        for lb,el,az in [("Front",0,0),("Side",0,-90),("Top",90,-90),("Iso",25,-60)]:
            tk.Button(vf,text=lb,font=self.font_small,bg=BG_DARK,fg=TEXT_LIGHT,bd=0,padx=8,pady=3,cursor="hand2",
                      command=lambda e=el,a=az:self._set_view(e,a)).pack(side="left",expand=True,fill="x",padx=2)
        # DH table
        ttk.Separator(panel,orient="horizontal").pack(fill="x",pady=10)
        tk.Label(panel,text="📋  DH Parameters",font=self.font_label,bg=BG_PANEL,fg=TEXT_LIGHT).pack(anchor="w",pady=(0,4))
        df=tk.Frame(panel,bg=BG_DARK,bd=1,relief="solid",padx=6,pady=4); df.pack(fill="x")
        for c,h in enumerate(["Link","θ","d","a","α"]):
            tk.Label(df,text=h,font=self.font_small,bg=BG_DARK,fg=TEXT_DIM,width=8,anchor="center").grid(row=0,column=c,padx=1,pady=1)
        L1,L2,L3=LINK_LENGTHS
        for i,row in enumerate([("1","θ₁","0","0","90°"),("2","θ₂","0",f"{L1}","0°"),
                                 ("3","θ₃","0",f"{L2}","0°"),("4","θ₄","0",f"{L3}","0°")]):
            for c,v in enumerate(row):
                fg=JOINT_COLORS[i] if c==0 else TEXT_LIGHT
                tk.Label(df,text=v,font=self.font_small,bg=BG_DARK,fg=fg,width=8,anchor="center").grid(row=i+1,column=c,padx=1,pady=1)

    def _draw(self):
        ax=self.ax; ax.clear(); tr=sum(LINK_LENGTHS); lim=tr*0.72
        ax.set_xlim(-lim,lim); ax.set_ylim(-lim,lim); ax.set_zlim(-tr*0.1,tr*1.05+BASE_HEIGHT)
        ax.set_facecolor(BG_CANVAS)
        ax.set_xlabel("X",color=ACCENT,fontsize=9); ax.set_ylabel("Y",color=ACCENT2,fontsize=9); ax.set_zlabel("Z",color=ACCENT3,fontsize=9)
        ax.tick_params(colors=TEXT_DIM,labelsize=7)
        for p in [ax.xaxis.pane,ax.yaxis.pane,ax.zaxis.pane]: p.fill=False; p.set_edgecolor(GRID_COLOR)
        ax.grid(True,color=GRID_COLOR,linewidth=0.4,alpha=0.5)
        ax.view_init(elev=self.view_elev,azim=self.view_azim)
        # Ground grid
        gr=np.linspace(-lim,lim,12)
        for g in gr:
            ax.plot([g,g],[-lim,lim],[0,0],color=GRID_COLOR,linewidth=0.3,alpha=0.3)
            ax.plot([-lim,lim],[g,g],[0,0],color=GRID_COLOR,linewidth=0.3,alpha=0.3)
        # Pedestal (column from ground to base)
        bh=BASE_HEIGHT
        ax.plot([0,0],[0,0],[0,bh],color=TEXT_DIM,linewidth=6,alpha=0.5,solid_capstyle='round')
        ax.plot([0,0],[0,0],[0,bh],color='#475569',linewidth=4,solid_capstyle='round',zorder=4)
        # Base disc at BASE_HEIGHT
        tb=np.linspace(0,2*np.pi,30); xb=22*np.cos(tb); yb=22*np.sin(tb); zb=np.full_like(tb,bh)
        ax.plot(xb,yb,zb,color=TEXT_DIM,linewidth=1.5,alpha=0.7)
        ax.add_collection3d(Poly3DCollection([list(zip(xb,yb,zb))],color='#334155',alpha=0.6,edgecolor=TEXT_DIM,linewidth=0.5))
        # Ground disc (footprint)
        zb0=np.zeros_like(tb)
        ax.plot(xb,yb,zb0,color=TEXT_DIM,linewidth=1,alpha=0.3)
        ax.add_collection3d(Poly3DCollection([list(zip(xb,yb,zb0))],color='#1e293b',alpha=0.3,edgecolor=TEXT_DIM,linewidth=0.3))
        # Rotation arc at BASE_HEIGHT
        if abs(self.thetas[0])>1:
            at=np.linspace(0,math.radians(self.thetas[0]),40); ax.plot(30*np.cos(at),30*np.sin(at),np.full_like(at,bh),color=JOINT_COLORS[0],linewidth=2,alpha=0.6)
        # Axes
        al=40
        ax.quiver(0,0,0,al,0,0,color=ACCENT,arrow_length_ratio=0.1,linewidth=1.5,alpha=0.7)
        ax.quiver(0,0,0,0,al,0,color=ACCENT2,arrow_length_ratio=0.1,linewidth=1.5,alpha=0.7)
        ax.quiver(0,0,0,0,0,al,color=ACCENT3,arrow_length_ratio=0.1,linewidth=1.5,alpha=0.7)
        ax.text(al+5,0,0,"X",color=ACCENT,fontsize=8); ax.text(0,al+5,0,"Y",color=ACCENT2,fontsize=8); ax.text(0,0,al+5,"Z",color=ACCENT3,fontsize=8)
        # FK
        pts=forward_kinematics_3d(self.thetas)
        lw=[5,4,3]
        for i in range(self.NL):
            x0,y0,z0=pts[i]; x1,y1,z1=pts[i+1]
            ax.plot([x0,x1],[y0,y1],[z0,z1],color=LINK_COLORS[i],linewidth=lw[i]+4,alpha=0.15,solid_capstyle="round")
            ax.plot([x0,x1],[y0,y1],[z0,z1],color=LINK_COLORS[i],linewidth=lw[i],solid_capstyle="round",zorder=5)
        # Joints
        jsz=[120,100,80]; jlbl=["J1","J2","J3"]
        for i in range(self.NL):
            x,y,z=pts[i]
            ax.scatter([x],[y],[z],c=JOINT_COLORS[i],s=jsz[i],zorder=10,edgecolors='white',linewidths=1.5,depthshade=False)
            ax.text(x+8,y+8,z+8,jlbl[i],color=JOINT_COLORS[i],fontsize=7,fontweight='bold')
        # EE
        ex,ey,ez=pts[-1]
        ax.scatter([ex],[ey],[ez],c=ACCENT3,s=100,marker='D',zorder=10,edgecolors='white',linewidths=2,depthshade=False)
        ax.text(ex+10,ey+10,ez+10,"EE",color=ACCENT3,fontsize=8,fontweight='bold')
        # Trail
        self.trail_points.append(pts[-1])
        if len(self.trail_points)>1000: self.trail_points=self.trail_points[-1000:]
        if len(self.trail_points)>1:
            trail=np.array(self.trail_points)
            ax.plot(trail[:,0],trail[:,1],trail[:,2],color=ACCENT3,linewidth=1.8,alpha=0.8)
        # Projection
        ax.plot([ex,ex],[ey,ey],[0,ez],'--',color=TEXT_DIM,linewidth=0.8,alpha=0.4)
        ax.scatter([ex],[ey],[0],c=TEXT_DIM,s=30,alpha=0.3,marker='x',depthshade=False)
        # Target
        if self.target:
            tx,ty,tz=self.target
            ax.scatter([tx],[ty],[tz],c=TARGET_COLOR,s=200,marker='*',zorder=15,edgecolors='white',linewidths=0.8,depthshade=False)
            ax.text(tx+12,ty+12,tz+12,"TARGET",color=TARGET_COLOR,fontsize=8,fontweight='bold')
            ax.plot([ex,tx],[ey,ty],[ez,tz],'--',color=TARGET_COLOR,linewidth=1.2,alpha=0.6)
        ax.set_title("3D Robot Arm — Base at Origin",color=TEXT_LIGHT,fontsize=12,fontweight='bold',pad=8)
        self.fig.tight_layout(pad=1.0); self.canvas.draw_idle()
        # Info
        dist=math.sqrt(ex**2+ey**2+ez**2)
        self.lbl_ee_x.config(text=f"X : {ex:>+8.2f} mm"); self.lbl_ee_y.config(text=f"Y : {ey:>+8.2f} mm")
        self.lbl_ee_z.config(text=f"Z : {ez:>+8.2f} mm"); self.lbl_ee_d.config(text=f"Dist: {dist:>8.2f} / Max: {tr} mm")
        if self.target:
            tx,ty,tz=self.target; err=math.sqrt((ex-tx)**2+(ey-ty)**2+(ez-tz)**2)
            self.lbl_tgt.config(text=f"Target X:{tx:>+.1f} Y:{ty:>+.1f} Z:{tz:>+.1f}")
            self.lbl_ik_err.config(text=f"IK Error: {err:.2f} mm")
            st=("✓ Converged!",SUCCESS_CLR) if err<1 else ("≈ Close",WARN_CLR) if err<5 else ("✗ Not converged",ACCENT)
            self.lbl_ik_st.config(text=st[0],fg=st[1])
        else:
            self.lbl_tgt.config(text="No target set"); self.lbl_ik_err.config(text=""); self.lbl_ik_st.config(text="")

    def _on_slider(self,idx,val):
        self.thetas[idx]=float(val); self.angle_labels[idx].config(text=f"{float(val):+.1f}°"); self._draw()

    def _solve_ik(self):
        try: 
            xc=float(self.target_entries["Xc"].get())
            yc=float(self.target_entries["Yc"].get())
            zc=float(self.target_entries["Zc"].get())
            r=float(self.target_entries["R"].get())
        except ValueError: self.lbl_ik_st.config(text="⚠ Nhập Xc,Yc,Zc,R hợp lệ!",fg=WARN_CLR); return
        
        if self.anim_id:
            self.root.after_cancel(self.anim_id)
            
        self.trail_points.clear()
        # Move immediately to starting point of the circle
        tx0 = xc + r * math.cos(0)
        ty0 = yc + r * math.sin(0)
        tz0 = zc
        self.target = (tx0, ty0, tz0)
        result_start, err = inverse_kinematics_3d(self.target, self.thetas)
        
        start_thetas = list(self.thetas)
        n_init_steps = 30
        init_step = [0]
        
        def _trace():
            n_steps = 100
            step = [0]
            def _circle_step():
                if step[0] > n_steps: return
                theta = (step[0] / n_steps) * 2 * math.pi
                tx = xc + r * math.cos(theta)
                ty = yc + r * math.sin(theta)
                tz = zc
                
                self.target = (tx, ty, tz)
                result, err = inverse_kinematics_3d(self.target, self.thetas)
                self.ik_error = err
                
                for i in range(self.NJ):
                    self.thetas[i] = result[i]
                    self.sliders[i].set(result[i])
                    self.angle_labels[i].config(text=f"{result[i]:+.1f}°")
                self._draw()
                step[0] += 1
                if step[0] <= n_steps:
                    self.anim_id = self.root.after(30, _circle_step)
            _circle_step()

        def _init_move():
            if init_step[0] >= n_init_steps: 
                _trace()
                return
            t = (init_step[0] + 1) / n_init_steps
            te = t * t * (3 - 2 * t)
            for i in range(self.NJ):
                self.thetas[i] = start_thetas[i] + (result_start[i] - start_thetas[i]) * te
                self.sliders[i].set(self.thetas[i])
                self.angle_labels[i].config(text=f"{self.thetas[i]:+.1f}°")
            self._draw()
            init_step[0] += 1
            if init_step[0] <= n_init_steps:
                self.anim_id = self.root.after(25, _init_move)
                
        _init_move()

    def _solve_line_ik(self):
        try:
            tx = float(self.line_entries["X"].get())
            ty = float(self.line_entries["Y"].get())
            tz = float(self.line_entries["Z"].get())
        except ValueError: self.lbl_ik_st.config(text="⚠ Nhập X,Y,Z hợp lệ!",fg=WARN_CLR); return
        
        if self.anim_id:
            self.root.after_cancel(self.anim_id)
            
        x0, y0, z0 = forward_kinematics_3d(self.thetas)[-1]
        self.target = (tx, ty, tz)
        
        dist = math.sqrt((tx-x0)**2 + (ty-y0)**2 + (tz-z0)**2)
        n_steps = max(30, int(dist / 1.5)) # Cách ~1.5mm sẽ có 1 điểm nội suy
        step = [0]
        
        def _line_step():
            if step[0] > n_steps: return
            t = step[0] / n_steps
            cx = x0 + (tx - x0) * t
            cy = y0 + (ty - y0) * t
            cz = z0 + (tz - z0) * t
            
            result, err = inverse_kinematics_3d((cx, cy, cz), self.thetas)
            self.ik_error = err
            
            for i in range(self.NJ):
                self.thetas[i] = result[i]
                self.sliders[i].set(result[i])
                self.angle_labels[i].config(text=f"{result[i]:+.1f}°")
            self._draw()
            
            step[0] += 1
            if step[0] <= n_steps:
                self.anim_id = self.root.after(30, _line_step)
                
        _line_step()

    def _animate_ik(self):
        pass

    def _reset(self):
        self.thetas=[0.0,60.0,-30.0,0.0]; self.trail_points.clear()
        for i in range(self.NJ): self.sliders[i].set(self.thetas[i]); self.angle_labels[i].config(text=f"{self.thetas[i]:+.1f}°")
        self._draw()

    def _clear_target(self): self.target=None; self._draw()
    def _clear_trail(self): self.trail_points.clear(); self._draw()
    def _set_view(self,e,a): self.view_elev=e; self.view_azim=a; self._draw()

if __name__=="__main__":
    root=tk.Tk(); app=RobotArm3DSimulator(root); root.mainloop()
