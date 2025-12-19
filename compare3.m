[mt,failrate,c,mt1,failrate1,c1,mt2,failrate2,c2,B,M,cloud,B1,M1,cloud1,B2,M2,cloud2] = mian(440,600);
function [mt,failrate,c,mt1,failrate1,c1,mt2,failrate2,c2,B,M,cloud,B1,M1,cloud1,B2,M2,cloud2] = mian(m,n)
    for i = m : 40 : n
        M=createM(20);
        B=createB(i);          
        cloud=createCloud(1);
        M1=M;
        M2=M;
        B1=B;
        B2=B;
        cloud1=cloud;
        cloud2=cloud;
        B=resort(B);
        B1=resort1(B1);
        B2=resort2(B2);
        B2=rtcB(B2,cloud2);
        [B,M,cloud]=Algorithm1(B,M,cloud);
        [B1,M1,cloud1]=COFE(B1,M1,cloud1);
        [B2,M2,cloud2]=DCDS(B2,M2,cloud2);
        [mt((i-m)/40+1),failrate((i-m)/40+1),c((i-m)/40+1),cloud] = result(B,M,cloud);
        [mt1((i-m)/40+1),failrate1((i-m)/40+1),c1((i-m)/40+1),cloud1] = result(B1,M1,cloud1);
        [mt2((i-m)/40+1),failrate2((i-m)/40+1),c2((i-m)/40+1),cloud2] = result(B2,M2,cloud2);
    end
end

function [mt,failrate,c,cloud] = result(B,M,cloud)
    c=0;
    fail=0;
    w=0;
    for i = 1:size(B,2)
        if B(i).nodes(size(B(i).nodes,1)).ft == 0
            fail=fail+1;
        end
        failrate = fail/size(B,2);
    end
    mt=0;
    for i = 1:size(B,2)
        if B(i).nodes(size(B(i).nodes,1)).ft ~= 0
            for j = 1:size(B(i).edges,1)
                pre = B(i).nodes(B(i).edges(j).pre+1).where ; 
                suc = B(i).nodes(B(i).edges(j).suc+1).where;
                if pre == 0 || suc == 0   %At least one of the two tasks must be on the vehicle.
                    if pre ~= suc         %One on the edge server, the other on the vehicle
                        if suc == 0       
                            temp=pre;
                            pre=suc;
                            suc=temp;
                        end
                        if suc == size(M,2)+1   %On the car and cloud server respectively
                            mt = mt + 2;
                        else                   %%On the vehicle and edge server respectively
                            if abs(findcarpos(B,i,B(i).nodes(pre+1).ft) - M(suc).pos)>100    %The edge server is not within the vehicle's communication range.
                                mt=mt+2;
                            elseif abs(findcarpos(B,i,B(i).nodes(pre+1).ft) - M(suc).pos)<=100  %infra metas
                                mt=mt+1;
                            end
                        end
                    end
                end
                if pre ~= 0 && suc ~= 0 && pre ~= suc  %Edge servers and cloud servers or edge servers
                    mt=mt+1;
                end
            end
        end
    end
    mt=mt/(size(B,2)-fail);
    for i = 1:size(B,2)
        if B(i).nodes(size(B(1).nodes,1)).ft ~= 0
            for j = 1:size(B(i).nodes,1)-1
                w=w+B(i).nodes(j).comp;
                c=c+transenergy(i,j,B,M) + scheduleenergy(i,j,B,M,cloud);
            end
        end
    end
    c=c/w*5000;
end

function x = scheduleenergy(i,j,B,M,cloud)
    if B(i).nodes(j+1).where == 0
        x=B(i).nodes(j+1).comp / B(i).f * B(i).c;
    elseif B(i).nodes(j+1).where == size(M,2)+1
        x=B(i).nodes(j+1).comp / cloud.f * cloud.c;
    else
        x=B(i).nodes(j+1).comp / M(B(i).nodes(j+1).where).f * M(B(i).nodes(j+1).where).c;
    end
end

function x=transenergy(i,j,B,M)
    x=0;
    if j == 0
        x=x+0;
    else
        pre = findpre(B,i,j);
        for n = pre
            index=[B(i).edges.pre] == n & [B(i).edges.suc] == j;
            if B(i).nodes(n+1).where == B(i).nodes(j+1).where   %%Two tasks on the same edge server or vehicle
                x=x+0;
            end
            if (B(i).nodes(n+1).where == 0 || B(i).nodes(j+1).where == 0) && B(i).nodes(n+1).where ~= B(i).nodes(j+1).where %One is on the vehicle, the other is on the edge server
                if B(i).nodes(n+1).where == 0
                    if B(i).nodes(j+1).where == size(M,2)+1 %Car to cloud server
                        x = x + B(i).edges(index).trans * (3 * 7.81 * 10^-9 + 10 * 80 * 10^-9);
                    else         %Car to edge server
                        if abs(findcarpos(B,i,B(i).nodes(n+1).ft) - M(B(i).nodes(j+1).where).pos) > 100
                            x=x+B(i).edges(index).trans * (5 *17.77*10^-9 +3 * 7.81 * 10^-9);
                        else
                            x=x+B(i).edges(index).trans * 3 * 7.81 * 10^-9;
                        end
                    end
                else
                    if B(i).nodes(n+1).where == size(M,2)+1 %Edge servers to cloud servers
                        x = x + B(i).edges(index).trans * (3 * 7.81 * 10^-9 + 10 * 80 * 10^-9);
                    else
                        if abs(findcarpos(B,i,B(i).nodes(n+1).ft) - M(B(i).nodes(n+1).where).pos) > 100
                            x=x+B(i).edges(index).trans * (5 *17.77*10^-9 +3 * 7.81 * 10^-9);
                        else
                            x=x+B(i).edges(index).trans * 5 *17.77*10^-9;
                        end
                    end
                end
            end
            if B(i).nodes(n+1).where ~= 0 && B(i).nodes(j+1).where ~= 0 && B(i).nodes(n+1).where ~= B(i).nodes(j+1).where
                if B(i).nodes(n+1).where == size(M,2)+1 || B(i).nodes(j+1).where == size(M,2)+1   %%Edge servers to cloud servers
                    x=x+B(i).edges(index).trans * 10 * 80 * 10^-9;
                else        %Edge servers to Edge servers
                    x=x+B(i).edges(index).trans * 5 * 17.77 * 10^-9;
                end
            end
        end
    end
end

function [B,M,cloud]=Algorithm1(B,M,cloud)
    [x,Border]=sort([B.r]);     %Get application scheduling order
    for i = Border   
        B1=B;
        M1=M;
        [x,norder]=sort([B(1).nodes.rank],"descend");
        for j = norder-1
            [B,M,cloud]=Algorithm2(B,M,i,j,cloud);
            if B(i).nodes(j+1).where == -1
                B=B1;
                M=M1;
                break;
            end
        end
    end
end

function [B,M,cloud]=Algorithm2(B,M,i,j,cloud)   %Schedule the j-th task of the i-th application
    B1=B;
    M1=M;
    if j==size(B(i).nodes,1)-1     %For dummy task termination, the scheduler is on the vehicle
        B(i).nodes(j+1).st=B(i).req;
        B(i).nodes(j+1).ft=B(i).req;
        B(i).nodes(j+1).statue=1;
        B(i).nodes(j+1).where=0;
        B(i).schedulelist=[i,j;B(i).schedulelist];
    elseif j==0                    %For dummy task start tasks, scheduling is performed on the vehicle
        st=inf;
        x=findcarpos(B,i,subdeadline(B,i,j));  %For dummy task start tasks, scheduling is performed on the vehicle
        suc=findsuc(B,i,j);
        for n = suc
            trans = B(i).edges([B(i).edges.pre] == j & [B(i).edges.suc] == n).trans;
            t = B(i).nodes(n+1).st - transtime(0,B(i).nodes(n+1).where,trans,x,M);
            if t < st
                st=t;
            end
        end
        B(i).nodes(j+1).st=st;
        B(i).nodes(j+1).ft=st;
        B(i).nodes(j+1).statue=1;
        B(i).nodes(j+1).where=0;
        B(i).schedulelist=[i,j;B(i).schedulelist];
    else                          %For real tasks
        st = -inf;
        t=subdeadline(B,i,j);      
        x=findcarpos(B,i,t);       
        suc=findsuc(B,i,j);
        if suc==size(B(i).nodes,1)-1  %%The children task is a virtual task  
            E1=[];
        else
            E1=unique(findsucpos(B,i,j));  %The children task is the one that needs to be scheduled
        end
        E2 = setdiff(findedgecar(M,x),E1);
        E3 = setdiff(setdiff(0:size(M,2),E2),E1);
        if ~isempty(E1)
            [st,B,M,cloud]=Algorithm3(i,j,E1,st,B,M,cloud);
        end
        if st == -inf
            if ~isempty(E2)
                [st,B,M,cloud]=Algorithm3(i,j,E2,st,B,M,cloud);
            end
            if st == -inf
                if ~isempty(E3)
                    [st,B,M,cloud]=Algorithm3(i,j,E3,st,B,M,cloud);
                end
                if st == -inf
                    [st,B,M,cloud]=Algorithm3(i,j,size(M,2)+1,st,B,M,cloud);
                end
            end
        end
    end
end

function [st,B,M,cloud]=Algorithm3(i,j,E,st,B,M,cloud)
    st = -inf;     %%Failed to schedule and returned-inf
    ft = -inf;
    pos=-1;
    tasknum=0;
    for n=1:size(E,2)
        [st1,ft1,num,cloud]=schedule(i,j,E(n),B,M,cloud);
        if st1 == -inf
            [st1,ft1,num,B,M,cloud] = Algorithm4(i,j,E(n),st1,ft1,num,B,M,cloud);
        end
        if st1>st
            st=st1;
            ft=ft1;
            pos=E(n);
            tasknum=num;
        end
    end
    if pos == 0 
        B(i).nodes(j+1).st=st;
        B(i).nodes(j+1).ft=ft;
        B(i).nodes(j+1).statue=1;
        B(i).nodes(j+1).where=pos;
        B(i).schedulelist=[B(i).schedulelist(1:tasknum,:);i,j;B(i).schedulelist(tasknum+1:end,:)];
    elseif pos > 0 && pos < size(M,2)+1
        B(i).nodes(j+1).st=st;
        B(i).nodes(j+1).ft=ft;
        B(i).nodes(j+1).statue=1;
        B(i).nodes(j+1).where=pos;
        M(pos).schedulelist=[M(pos).schedulelist(1:tasknum,:);i,j;M(pos).schedulelist(tasknum+1:end,:)];
    elseif pos == size(M,2)+1
        if st >= B(i).r && ft <= B(i).req
            B(i).nodes(j+1).st=st;
            B(i).nodes(j+1).ft=ft;
            B(i).nodes(j+1).statue=1;
            B(i).nodes(j+1).where=pos;
            cloud.schedulelist=[cloud.schedulelist;i,j];
        end
    end
end

function [st,ft,num,cloud]=schedule(i,j,Ea,B,M,cloud)
    if Ea ~= size(M,2)+1
        num=-1;
        st = -inf;
        ft = -inf;
        deadline = deadlineforM(i,j,Ea,B,M);
        if Ea == 0 
            list = B(i).schedulelist;
            scheduletime = B(i).nodes(j+1).comp / B(i).f;
        else
            list = M(Ea).schedulelist;
            scheduletime = B(i).nodes(j+1).comp / M(Ea).f;
        end
        timelist=[0,0];
        for m=1:size(list,1)
            timelist = [timelist;B(list(m,1)).nodes(list(m,2)+1).st,B(list(m,1)).nodes(list(m,2)+1).ft];
        end
        if size(timelist,1)>1
            for n = 2:size(timelist,1)
                if timelist(n,1) <= deadline
                    if timelist(n,1)-scheduletime >= timelist(n-1,2) && timelist(n,1)-scheduletime >= B(i).r
                        st = timelist(n,1)-scheduletime;
                        ft = st+scheduletime;
                        num=n-2;
                    end
                else
                    n=n-1;
                    break;
                end
            end
        else
            n=1;
        end
        if (deadline - scheduletime) >= timelist(n,2) && (deadline - scheduletime) >= B(i).r
            st = deadline-scheduletime;
            ft = deadline;
            num=n-1;
        end
    elseif Ea == size(M,2)+1
        st = inf;
        ft = inf;
        num = -1;
        for suc = findsuc(B,i,j)
            trans = B(i).edges([B(i).edges.pre] == j & [B(i).edges.suc] == suc).trans;
            st1 = round(B(i).nodes(suc+1).st - transtime(B(i).nodes(suc+1).where,Ea,trans,findcarpos(B,i,subdeadline(B,i,j)),M) - B(i).nodes(j+1).comp / cloud.f,3);
            ft1 = round(st1 + B(i).nodes(j+1).comp / cloud.f,3);
            if st > st1
                st=st1;
                ft=ft1;
            end
        end

    end
end

function [st,ft,num,B,M,cloud]=Algorithm4(i,j,Ea,st,ft,num,B,M,cloud)
    if Ea == size(M,2)+1
        return;
    end
    B1=B;
    M1=M;
    num1=num;
    deadline = deadlineforM(i,j,Ea,B,M);
    if Ea == 0 
        list = B(i).schedulelist;     %%forward scheduling table
        scheduletime = B(i).nodes(j+1).comp / B(i).f;
    else
        list = M(Ea).schedulelist;    %%forward scheduling table
        scheduletime = B(i).nodes(j+1).comp / M(Ea).f;
    end
    timelist=[0,0];
    for h=1:size(list,1)
        timelist = [timelist;B(list(h,1)).nodes(list(h,2)+1).st,B(list(h,1)).nodes(list(h,2)+1).ft];
    end
    for m = size(timelist,1):-1:2    %%m-1 is the subscript of the current task
        if timelist(m,2)<=deadline
            if timelist(m,2)-scheduletime>=timelist(m-1,2) && timelist(m,2)-scheduletime>=B(i).r
                st = timelist(m,2)-scheduletime;
                ft = timelist(m,2);
                if Ea == 0 
                    B(i).nodes(j+1).st=st;
                    B(i).nodes(j+1).ft=ft;
                    B(i).nodes(j+1).statue=1;
                    B(i).nodes(j+1).where=Ea;
                    B(i).schedulelist(m-1,1)=i;
                    B(i).schedulelist(m-1,2)=j;
                elseif Ea > 0
                    B(i).nodes(j+1).st=st;
                    B(i).nodes(j+1).ft=ft;
                    B(i).nodes(j+1).statue=1;
                    B(i).nodes(j+1).where=Ea;
                    M(Ea).schedulelist(m-1,1)=i;
                    M(Ea).schedulelist(m-1,2)=j;
                end
                [st1,ft1,num,cloud]=schedule(list(m-1,1),list(m-1,2),Ea,B,M,cloud);
                if st1 ~= -inf && ft1<ft
                    pre=findpre(B,list(m-1,1),list(m-1,2));
                    flag=1;
                    for n = pre
                        x=[B(list(m-1,1)).edges.pre] == n & [B(list(m-1,1)).edges.suc] == list(m-1,2);
                        if B(list(m-1,1)).nodes(list(m-1,2)+1).ft + transtime(B(list(m-1,1)).nodes(n+1).where,B(list(m-1,1)).nodes(list(m-1,2)+1).where,B(list(m-1,1)).edges(x).trans,findcarpos(B,list(m-1,1),subdeadline(B,list(m-1,1),list(m-1,2))),M) > st1
                            flag=0;
                        end
                    end
                    if flag==1
                        if Ea == 0 
                            B(list(m-1,1)).nodes(list(m-1,2)+1).st=st1;
                            B(list(m-1,1)).nodes(list(m-1,2)+1).ft=ft1;
                            B(list(m-1,1)).nodes(list(m-1,2)+1).statue=1;
                            B(list(m-1,1)).nodes(list(m-1,2)+1).where=Ea;
                            B(list(m-1,1)).schedulelist=[B(list(m-1,1)).schedulelist(1:num,:);list(m-1,1),list(m-1,2);B(list(m-1,1)).schedulelist(num+1:end,:)];
                        elseif Ea > 0
                            B(i).nodes(j+1).st=st1;
                            B(i).nodes(j+1).ft=ft1;
                            B(i).nodes(j+1).statue=1;
                            B(i).nodes(j+1).where=Ea;
                            M(Ea).schedulelist=[M(Ea).schedulelist(1:num,:);list(m-1,1),list(m-1,2);M(Ea).schedulelist(num+1:end,:)];
                        end
                    end
                end
            end
        end
    end
    if st == -inf
        B=B1;
        M=M1;
        st=-inf;
        ft=-inf;
        num=num1;
    else
        if st1 == -inf || ft1>=ft
            B=B1;
            M=M1;
            st=-inf;
            ft=-inf;
            num=num1;
        else
            if flag==0
                B=B1;
                M=M1;
                st=-inf;
                ft=-inf;
                num=num1;
            end
        end
    end
end

function t=deadlineforM(i,j,Ea,B,M)    %%Deadline for task on device Ma
    suc=findsuc(B,i,j);
    t=inf;
    for m = suc
        if B(i).nodes(m+1).where ~= Ea
            n = [B(i).edges.pre] == j & [B(i).edges.suc] == m;
            x=round(B(i).nodes(m+1).st - transtime(B(i).nodes(m+1).where,Ea,B(i).edges(n).trans,findcarpos(B,i,subdeadline(B,i,j)),M),3);
        else
            x=round(B(i).nodes(m+1).st,3);
        end
        if t>x
            t=x;
        end
    end
end

function suc=findsuc(B,i,a)   %%Find the children task of task a in the i-th task graph
    suc=[];
    for j = 1 : size(B(i).edges,1)
        if a == B(i).edges(j).pre
            suc = [suc,B(i).edges(j).suc];
        end
    end
end

function pre=findpre(B,i,a)  %Find the parents task of task a in the i-th task graph
    pre=[];
    for j = 1 : size(B(i).edges,1)
        if a == B(i).edges(j).suc
            pre = [pre,B(i).edges(j).pre];
        end
    end
end

function rank=rerank(B,i,a)  %Calculate the rank for task a in task graph i
    if a == size(B(i).nodes,1) - 1
        rank = B(i).req;
    else
        suc = findsuc(B,i,a);
        rank = inf;
        for m = suc
            j=[B(i).edges.pre] == a & [B(i).edges.suc] == m;
            x=B(i).nodes(m+1).rank-B(i).nodes(m+1).comp/5000-B(i).edges(j).trans*17.77*10^-9;
            if rank > x
                rank = x;
            end
        end
    end
end

function t=subdeadline(B,i,j)    %Sub deadline
    suc=findsuc(B,i,j);
    t=inf;
    for m = suc
        n = [B(i).edges.pre] == j & [B(i).edges.suc] == m;
        x=round(B(i).nodes(m+1).st-B(i).edges(n).trans*17.77*10^-9,3);
        if t>x
            t=x;
        end
    end
end

function x=findcarpos(B,i,a)   %Find the vehicle's position at the a-th second
    mu=B(i).r;
    sigma=10;
    target_peak = B(i).v;
    original_peak = 1 / (sigma * sqrt(2*pi));  
    k = target_peak / original_peak;           
    xlabel = linspace(0, 10, 1000);
    f = @(xlabel) k * normpdf(xlabel, mu*rand()*2, sigma);  
    x = integral(f, B(i).r, a)+B(i).xlabel; 
%     x=B(i).xlabel+(a-B(i).r)*B(i).v;
    if x<0
        x=0;
    end
    if x>1100
        x=1100;
    end
end

function x=findedgecar(M,a)  %%Identify the edge server and vehicle that can communicate at location a
    x=[0];
    for i = 1 : size(M,2)
        if abs(M(i).pos-a) <= 100
            x=[x,M(i).order];
        end
    end

end

function t = transtime(sa,sb,trans,x,M)    %%Transfer time between two devices
    if sa ~= sb && sa ~= 0 && sb ~= 0  %%Edge to Edge or Cloud
        if sa == size(M,2)+1 || sb == size(M,2)+1
            t = round(trans * 80 * 10^-9,3);
        else
            t = round(trans * 17.77 * 10^-9,3);
        end
    end
    if (sa ~= sb)
        if (sa == 0 && sb ~= 0) || (sa ~= 0 && sb == 0)
            if sa ~= 0
                sb = sa;
                sa =0;
            end
            if sb == size(M,2)+1
                t=round(trans * 7.81 * 10^-9 + trans * 80 * 10^-9,3);
            else
                rangecar=findedgecar(M,x);
                rangecar=rangecar(2:end);
                if ismember(sb,rangecar)
                    t=round(trans * 7.81 * 10^-9 ,3);
                else
                    t = round(trans * (17.77 * 10^-9 + 7.81 * 10^-9),3);
                end
            end
        end
    end
    if sa == sb
        t=0;
    end

end

function sucpos=findsucpos(B,i,j)    %%Locate the children task
    sucpos=[];
    suc=findsuc(B,i,j);
    for n = suc
        sucpos=[sucpos,B(i).nodes(n+1).where];
    end
end

function B=resort(B)  %%sort
    for i = 1:size(B,2)
        for j = size(B(i).nodes,1)-1 : -1 : 0
            B(i).nodes(j+1).rank = round(rerank(B,i,j),3);
        end
    end
end

function [edges,nodes]=creategraph()  %Create a graph
    node = 1:10;
    layer0=[0];
    layer1 = node(1:3);
    layer2 = node(4:5);
    layer3 = node(6:7);
    layer4 = node(8:10);
    layer5=[11];
    n=1;
    for i=1:3
        edges(n)=struct('pre',0,'suc',i,'trans',(12+4*rand())*10^6/10);
        n=n+1;
    end

    % connects the first and second layers
    for i = layer1
        a=randi(2);
        len=length(layer2);
        x=randperm(len,a);
        for j=x
            edges(n) = struct("pre",i,'suc',layer2(j),'trans',(12+4*rand())*10^6/10);
            n=n+1;
        end
    end
    % Connect the second and third layers
    for i = layer2
        a=randi(2);
        len=length(layer3);
        x=randperm(len,a);
        for j=x
            edges(n) = struct("pre",i,'suc',layer3(j),'trans',(12+4*rand())*10^6/10);
            n=n+1;
        end
    end
    %Connect Layer 3 and Layer 4
    for i = layer3
        a=randi(2);
        len=length(layer4);
        x=randperm(len,a);
        for j=x
            edges(n) = struct("pre",i,'suc',layer4(j),'trans',(12+4*rand())*10^6/10);
            n=n+1;
        end
    end
    % Connect dummy tasks
    for i=layer4
        edges(n) = struct("pre",i,'suc',11,'trans',(12+4*rand())*10^6/10);
        n=n+1;
    end
    x=unique([edges.suc]);
    xx=setdiff(node,x);
    for m=xx
        edges(n) = struct("pre",0,'suc',m,'trans',(12+4*rand())*10^6/10);
        n=n+1;
    end
    edgestable=struct2table(edges);
    edgestable=sortrows(edgestable,[1,2]);
    edges=table2struct(edgestable);
    n=2;
    nodes(1)=struct('order',0,'comp',0,'rank',0,'st',0,'ft',0,'where',-1,'statue',0 ,'rtc',-1);
    for i = node
        nodes(n)=struct('order',i,'comp',fix(2000+1000*rand()/1)/10,'rank',0,'st',0,'ft',0,'where',-1,'statue',0 ,'rtc',-1);
        n=n+1;
    end
    nodes(n)=struct('order',n-1,'comp',0,'rank',0,'st',0,'ft',0,'where',-1,'statue',0 ,'rtc',-1);
    nodes=nodes';
end

function B=createB(a)%create user
    list=linspace(1,1000,a+1);
    list=list(2:end);
    xlabel_matrix = reshape(list, a/10, []);
    xlabel=[];
    for i =1:size(xlabel_matrix,1)
        xlabel=[xlabel,xlabel_matrix(i,:)];
    end
    for i = 1:a
        [edges,nodes]=creategraph();
        if mod(i,20)<=10
            flag=1;
        else
            flag=-1;
        end
        r=i*(10/a);
        B(i)=struct('order',i,'v',flag*randi([10,20]),'xlabel',xlabel(i),'edges',edges,'nodes',nodes,'r',round(r,3),'req',r+5,"schedulelist",[],"f",1000,'c',10,'key',[],"worktime",0);
        workload=0;
        for x = 1:size(B(i).nodes,1)
            workload=workload+B(i).nodes(x).comp;
        end
        req=workload/5000*2+r;
        B(i).req=round(req,3);
    end
end
function M=createM(a)  %Create edge servers
    p=[4000,4500,5000,5500,6000];
    c=[32,40.5,50,60.5,72];
    for i = 1: a
        x=mod(i,5);
        if x == 0
            x = 5;
        end
        M(i)=struct('order',i,'f',p(x),'c',c(x),'pos',50*i,"schedulelist",[],"worktime",0);
    end
end

function Cloud = createCloud(a)%Create a cloud server
    Cloud = struct('order',1,'f',20000,'c',300,'schedulelist',[]);
end

%%%-------------------------------------------------------------------------------------------------%%%

function [B,M,cloud] = COFE(B,M,cloud)
    B=resort1(B);
    B=findkeyroad(B);
    tolerace = 1e-10;
    for t = 0.001:0.001:30
        for i = 1:size(B,2)  %%Schedule the first task
            if abs(B(i).r - t) <=tolerace
                [B,M]=schedule1(i,0,t,B,M);
            end
        end
        for i = 1 : size(B,2)
            for j = 0 : (size(B(i).nodes,1)-2)
                if abs(t-B(i).nodes(j+1).ft)<tolerace  %%completion of task
                    if B(i).nodes(j+1).where == 0
                        B(i).schedulelist=B(i).schedulelist(2:end,:);
                    elseif B(i).nodes(j+1).where>0 && B(i).nodes(j+1).where<=size(M,2)
                        M(B(i).nodes(j+1).where).schedulelist = M(B(i).nodes(j+1).where).schedulelist(2:end,:);
                    elseif B(i).nodes(j+1).where == size(M,2) + 1
                        cloud.schedulelist = cloud.schedulelist(2:end,:);
                    end
                    T=findreadysuc(i,j,t,B);
                    for o = 1:size(T,1)
                        [B,M]=schedule1(T(o,1),T(o,2),t,B,M,cloud);
                    end
                end
            end
        end
    end
end

function [B,M] = schedule1(i,j,t,B,M,cloud)
    if j==0              %start dummy task
        B(i).nodes(j+1).st = B(i).r;
        B(i).nodes(j+1).ft = B(i).r;
        B(i).nodes(j+1).where = 0;
        B(i).nodes(j+1).statue = 1;
        B(i).schedulelist = [B(i).schedulelist;i,j];
        B(i).worktime = B(i).r;
    elseif j>=1 && j <= (size(B(i).nodes,1)-2)    %%real task
        ft = inf;
        pos=-1;
        for m = 0 : size(M,2)
            T=finishtime(i,j,m,t,B,M,cloud);
            if ft >= T && T<=B(i).req
                ft=T;
                pos=m;
            end
        end
        if pos == -1 && ft == inf  %%Terminate dummy task
            T = finishtime(i,j,size(M,2)+1,t,B,M,cloud);
            if ft >= T && T<=B(i).req
                ft = T;
                pos = size(M,2)+1;
            end
        end
        if pos==0
            B(i).nodes(j+1).st=round(ft-B(i).nodes(j+1).comp / B(i).f,3);
            B(i).nodes(j+1).ft=ft;
            B(i).nodes(j+1).where=pos;
            B(i).nodes(j+1).statue=1;
            B(i).schedulelist=[B(i).schedulelist;i,j];
            B(i).worktime=ft;
        elseif pos>0 && pos <= size(M,2)
            B(i).nodes(j+1).st=round(ft-B(i).nodes(j+1).comp / M(pos).f,3);
            B(i).nodes(j+1).ft=ft;
            B(i).nodes(j+1).where=pos;
            B(i).nodes(j+1).statue=1;
            M(pos).schedulelist = [M(pos).schedulelist;i,j];
            M(pos).worktime = ft ;
        elseif pos == size(M,2)+1
            B(i).nodes(j+1).st=round(ft-B(i).nodes(j+1).comp / cloud.f,3);
            B(i).nodes(j+1).ft=ft;
            B(i).nodes(j+1).where=pos;
            B(i).nodes(j+1).statue=1;
            cloud.schedulelist = [cloud.schedulelist;i,j];
        end
    else                 
        ft=finishtime(i,j,0,t,B,M);
        if ft <= B(i).req
            B(i).nodes(j+1).st=ft;
            B(i).nodes(j+1).ft=ft;
            B(i).nodes(j+1).where=0;
            B(i).nodes(j+1).statue=1;
            B(i).schedulelist=[B(i).schedulelist;i,j];
            B(i).worktime=ft;
        end
    end
end

function T = finishtime(i,j,a,t,B,M,cloud)
    pre = findpre(B,i,j);
    T=-1;
    for n = pre
        trans = B(i).edges([B(i).edges.pre] == n & [B(i).edges.suc] == j).trans;
        t1=B(i).nodes(n+1).ft + transtime(B(i).nodes(n+1).where,a,trans,findcarpos(B,i,t),M);
        if a == 0
            t1=max(t1,B(i).worktime)+B(i).nodes(j+1).comp / B(i).f;
        elseif a > 0 && a <= size(M,2)
            t1=max(t1,M(a).worktime)+B(i).nodes(j+1).comp / M(a).f;
        elseif a == size(M,2)+1
            t1=t1+B(i).nodes(j+1).comp / cloud.f;
        end
        if T<t1
            T=t1;
        end
    end
    T=round(T,3);
end

function T = findreadysuc(i,j,t,B)
    T=[];
    suc=findsuc(B,i,j);
    for n=suc
        flag=1;
        pre = findpre(B,i,n);
        idx=find(pre==j);
        pre(idx)=[]; 
        for m = pre
            if B(i).nodes(m+1).ft > t || B(i).nodes(m+1).ft == 0
                flag=0;
            end
        end
        if flag==1
            T=[T;i,n,B(i).nodes(n+1).rank];
        end
        if ~isempty(T)
            T=sortrows(T,3,'descend');
        end
    end
end

function B=findkeyroad(B)
    for i=1:size(B,2)
        x=[];
        [maxvalue,maxindex]=max([B(i).nodes.rank]);
        x=[x,maxindex];
        suc=findsuc(B,i,maxindex);
        ranks=B(i).nodes(suc).rank;
        [maxvalue1,maxindex1]=max(ranks);
        maxrow=suc(maxindex1);
        x=[x,maxrow];
        suc1=findsuc(B,i,maxrow);
        for j=1:size(suc1)
            if suc1(j)==(size(B(i).nodes,1)-1)
                suc1(j)=[];
            end
        end
        ranks1=B(i).nodes(suc1).rank;
        [maxvalue2,maxindex2]=max(ranks1);
        maxrow1=suc1(maxindex2);
        x=[x,maxrow1];
        suc2=findsuc(B,i,maxrow1);
        for j=1:size(suc2)
            if suc2(j)==(size(B(i).nodes,1)-1)
                suc2(j)=[];
            end
        end
        if ~isempty(suc2)
            ranks2=B(i).nodes(suc2).rank;
            [maxvalue3,maxindex3]=max(ranks2);
            maxrow2=suc2(maxindex3);
            x=[x,maxrow2];
        end
        B(i).key=x;
    end
end

function B=resort1(B)
    for i = 1:size(B,2)
        B(i).nodes(size(B(i).nodes,1)).rank=0;
        for j=size(B(i).nodes,1)-2:-1:1
            answer=rerank1(B,i,j);
            B(i).nodes(j+1).rank=answer;
        end
        B(i).nodes(1).rank=0;
    end
end

function answer=rerank1(B,i,j)  % Calculate task i,j's level
    x=0;
    answer=-1;
    if j==(size(B(i).nodes,1)-1)
        answer=0;
    else
        suc=findsuc(B,i,j);
        for m=suc
            if (1-1000^(-(B(i).nodes(j+1).comp/5000)))/(B(i).edges([B(i).edges.pre]==j & [B(i).edges.suc]==m).trans*17.77*10^-9)<rand(1)
                n=0;
            else
                n=1;
            end
            if x <= rerank1(B,i,m) + n*(B(i).edges([B(i).edges.pre]==j & [B(i).edges.suc]==m).trans*17.77*10^-9)
                x=rerank1(B,i,m) + n*(B(i).edges([B(i).edges.pre]==j & [B(i).edges.suc]==m).trans*17.77*10^-9);
                answer=rerank1(B,i,m) + n*(B(i).edges([B(i).edges.pre]==j & [B(i).edges.suc]==m).trans*17.77*10^-9)+B(i).nodes(j).comp/5000;
            end
        end
    end
end

function [B,M,cloud]=DCDS(B,M,cloud)
    [x,Border]=sort([B.r]); 
    for i = Border
        [x,norder]=sort([B(i).nodes.rank],"ascend");
        for j = 0 : norder -1
            [B,M,cloud]=schedule2(i,j,B,M,cloud);
            if B(i).nodes(j+1).ft == 0
                [B,M,cloud]=cDCDS(i,B,M,cloud);
                break;
            end
        end
    end
end

function [B,M,cloud]=cDCDS(i,B,M,cloud)
    Q=[];
    for x = 1 : size(B(i).nodes,1)-2
        ets(x,1) = i;
        ets(x,2) = x;
    end
    pre = findpre(B,i,size(B(i).nodes,1)-1);
    for j = pre
        flag=1;
        for n = pre
            pre1 = findpre(B,i,j);
            for nn = pre1
                if B(i).nodes(nn+1).statue == 0
                    flag=0;
                end
            end
        end
        est1 = inf;
        if flag == 1
            est1 = est(i,j,B,M,cloud);
        end
        trans = B(i).edges([B(i).edges.pre] == j & [B(i).edges.suc] == size(B(i).nodes,1)-1).trans;
        if flag == 0 || round(est1 + B(i).nodes(j+1).comp / cloud.f + trans * (80 *10^-9 + 7.81 * 10^-9),3) > B(i).req || B(i).nodes(j+1).statue==0
            Q=[i,j;Q];
            ets(ismember(ets,[i,j], 'rows'),:)=[];
        end
    end
    while ~isempty(Q)
        i=Q(1,1);
        j=Q(1,2);
        Q=Q(2:end,:); %i and j are the current tasks. Check if the parents task is added to the cloud server.
        if j == size(B(i).nodes,1)-1 || j == 0
            return
        else
            pre = findpre(B,i,j);   %%Check whether the parents node is scheduled to the cloud server
            for pre1=pre
                est2 = est(i,pre1,B,M,cloud);    %%Start time of the parents node on the cloud server
                trans = B(i).edges([B(i).edges.pre] == pre1 & [B(i).edges.suc] == j).trans;
                if round(B(i).nodes(pre1+1).ft + trans * (80 *10^-9),3) + B(i).nodes(pre1+1).rtc > B(i).req || B(i).nodes(pre1+1).statue == 0
                    Q = [i,pre1;Q];
                    ets(ismember(ets,[i,pre1], 'rows'),:)=[];
                end
            end
        end
    end
    [x,norder]=sort([B(i).nodes.rank],"ascend");
    for j = 0 : norder -1
        if ~ismember(ets,[i,j],'row')
            if j ~= 0
                [B,M,cloud]=schedulecloud(i,j,B,M,cloud);
            end
        end
    end
end

function [B,M,cloud]=schedulecloud(i,j,B,M,cloud)
    st=-inf;
    ft=-inf;
    pre = findpre(B,i,j);
    for n = pre
        if B(i).nodes(n+1).statue == 0
            return
        end
        trans = B(i).edges([B(i).edges.pre] == n & [B(i).edges.suc] == j).trans;
        ft1 = round(B(i).nodes(n+1).ft + transtime(B(i).nodes(n+1).where,size(M,2)+1,trans,findcarpos(B,i,B(i).nodes(j+1).ft),M)+B(i).nodes(n+1).comp / cloud.f,3);
        st1 = round(ft1 - B(i).nodes(n+1).comp / cloud.f,3);
        if ft < ft1
            st=st1;
            ft=ft1;
        end
    end
    if ft<B(i).req && ft ~= -inf
        B(i).nodes(j+1).st = st;
        B(i).nodes(j+1).ft = ft;
        B(i).nodes(j+1).where = size(M,2)+1;
        B(i).nodes(j+1).statue = 1;
        cloud.schedulelist = [i,j;cloud.schedulelist];
    end
end
function t=est(i,j,B,M,cloud)
    t=-inf;
    pre = findpre(B,i,j);
    for n = pre
        est1 = finishtime(i,j,size(M,2)+1,B(i).nodes(n+1).ft,B,M,cloud);
        if t < est1
            t=est1;
        end
    end
end

function x=rtc(i,j,B,cloud)
    if j == size(B(i).nodes,1)-1
        x=0;
    elseif j >= 0 && j<size(B(i).nodes,1)-1
        suc = findsuc(B,i,j);
        x=-inf;
        for n = size(suc,2):-1:1
            x1=rtce(i,j,suc(n),B,cloud) + B(i).nodes(j+1).comp / cloud.f;
            if x < x1
                x = x1;
            end
        end
    end
end

function x=rtce(i,j,n,B,cloud)
    if n == size(B(i).nodes,1)-1
        trans = B(i).edges([B(i).edges.pre] == j & [B(i).edges.suc] == n).trans;
        x = trans * (80 *10^-9 + 7.81 * 10^-9) + rtc(i,n,B,cloud);
    else
        x=rtc(i,n,B,cloud);
    end
end

function B=rtcB(B,cloud)
    for i = 1:size(B,2)
        for j = size(B(i).nodes,1)-1 : -1 : 0
            x=rtc(i,j,B,cloud);
            B(i).nodes(j+1).rtc = x;
        end
    end
end

function [B,M,cloud]=schedule2(i,j,B,M,cloud)
    if j == 0
        B(i).nodes(j+1).st=B(i).r;
        B(i).nodes(j+1).ft=B(i).r;
        B(i).nodes(j+1).where=0;
        B(i).nodes(j+1).statue=1;
        B(i).schedulelist=[i,j;B(i).schedulelist];
    elseif j > 0 && j<=(size(B(i).nodes,1)-2)
        flag=-1;
        flag1=-1;
        c=inf;
        st = 0;
        ft = 0;
        for m=0:size(M,2)
            [t,flag,st1,ft1] = EEST_LT(i,j,m,B,M);
            if flag ~= -1
                flag1=1;
                %Compare energy and assign a value to h
                if c > transenergy1(i,j,m,B,M) + scheduleenergy1(i,j,m,B,M,cloud)
                    c = transenergy1(i,j,m,B,M) + scheduleenergy1(i,j,m,B,M,cloud);
                    h=m;
                    st = st1;
                    ft = ft1;
                end
            end
        end
        if flag1 == -1
            tt = inf;
            for m=0:size(M,2)
                [t,flag,st1,ft1] = EEST_LT(i,j,m,B,M);
                %Select the one with the lowest energy
                if tt > t 
                    h = m;
                    st = st1;
                    ft = ft1;
                end
            end
        end
        if ft <= B(i).req
            if h > 0 && j<=(size(B(i).nodes,1)-2)
                B(i).nodes(j+1).st = st;
                B(i).nodes(j+1).ft = ft;
                B(i).nodes(j+1).where = h;
                B(i).nodes(j+1).statue = 1;
                M(h).worktime = ft;
            elseif h == 0 
                B(i).nodes(j+1).st = st;
                B(i).nodes(j+1).ft = ft;
                B(i).nodes(j+1).where = h;
                B(i).nodes(j+1).statue = 1;
                B(i).worktime = ft;
            end
        end
    elseif j > (size(B(i).nodes,1)-2)
        t = readytime(i,j,B);
        ft=finishtime(i,j,0,t,B,M);
        if ft <= B(i).req
            B(i).nodes(j+1).st=ft;
            B(i).nodes(j+1).ft=ft;
            B(i).nodes(j+1).where=0;
            B(i).nodes(j+1).statue=1;
            B(i).schedulelist=[B(i).schedulelist;i,j];
            B(i).worktime=ft;
        end
    end
end

function x=transenergy1(i,j,m,B,M)
    x=0;
    if j == 0
        x=x+0;
    else
        pre = findpre(B,i,j);
        for n = pre
            index=[B(i).edges.pre] == n & [B(i).edges.suc] == j;
            if B(i).nodes(n+1).where == m   %On the same edge server or vehicle
                x=x+0;
            end
            if (B(i).nodes(n+1).where == 0 || m == 0) && B(i).nodes(n+1).where ~= m %One is on the vehicle, the other is on the edge server
                if B(i).nodes(n+1).where == 0  
                    if m == size(M,2)+1 %Car to cloud server
                        x = x + B(i).edges(index).trans * (3 * 7.81 * 10^-9 + 10 * 80 * 10^-9);
                    else
                        if abs(findcarpos(B,i,B(i).nodes(n+1).ft) - M(m).pos) > 100
                            x=x+B(i).edges(index).trans * (5 *17.77*10^-9 +3 * 7.81 * 10^-9);
                        else
                            x=x+B(i).edges(index).trans * 3 * 7.81 * 10^-9;
                        end
                    end
                else 
                    if B(i).nodes(n+1).where == size(M,2)+1 %Car to edge server
                        x = x + B(i).edges(index).trans * (3 * 7.81 * 10^-9 + 10 * 80 * 10^-9);
                    else
                        if abs(findcarpos(B,i,B(i).nodes(n+1).ft) - M(B(i).nodes(n+1).where).pos) > 100
                            x=x+B(i).edges(index).trans * (5 *17.77*10^-9 +3 * 7.81 * 10^-9);
                        else
                            x=x+B(i).edges(index).trans * 5 *17.77*10^-9;
                        end
                    end
                end
            end
            if B(i).nodes(n+1).where ~= 0 || m ~= 0 || B(i).nodes(n+1).where ~= m %Both are on edge servers
                if B(i).nodes(n+1).where == size(M,2)+1 || m == size(M,2)+1 %Edge server to cloud server
                    x=x+B(i).edges(index).trans * 10 * 80 * 10^-9;
                else       %Edge server to edge server
                    x=x+B(i).edges(index).trans * 5 * 17.77 * 10^-9;
                end
            end
        end
    end
end

function x = scheduleenergy1(i,j,m,B,M,cloud)
    if m == 0
        x=B(i).nodes(j+1).comp / B(i).f * B(i).c;
    elseif m == size(M,2)+1
        x=B(i).nodes(j+1).comp / cloud.f * cloud.c;
    else
        x=B(i).nodes(j+1).comp / M(m).f *M(m).c;
    end
end

function lt=LT(i,j,B)
    lt=B(i).r + (B(i).req - B(i).r) * (B(i).nodes(1).rank - B(i).nodes(j+1).rank)/B(i).nodes(1).rank;
end

function [t,flag,st,ft]=EEST_LT(i,j,a,B,M)
    flag=a;
    readytime1=readytime(i,j,B);
    ft=round(finishtime(i,j,a,readytime1,B,M),3);
    if a==0
        st=round(ft-B(i).nodes(j+1).comp/B(i).f,3);
    else
        st=round(ft-B(i).nodes(j+1).comp/M(a).f,3);
    end
    suc = findsuc(B,i,j);
    t=-inf;
    for x = suc
        if x == (size(B(i).nodes,1)-1)
            transload = 0;
            if a ~= 0
                if abs(M(a).pos - findcarpos(B,i,readytime1)) <= 100
                    transload = 17.77*10^-9;
                else
                    transload =17.77*10^-9+7.81 * 10^-9;
                end
            end
            y=[B(i).edges.pre] == j & [B(i).edges.suc] == x;
            eest_lt = ft + B(i).edges(y).trans *transload - LT(i,x,B);
        else
            transload = 0;
            if a == 0
                for mpos = 1:size(M,2)
                    if abs(M(mpos).pos - findcarpos(B,i,readytime1)) <= 100
                        transload=transload+17.77*10^-9;
                    else
                        transload=transload+17.77*10^-9+7.81 * 10^-9;
                    end
                end
                transload = transload / size(M,2);
            elseif a > 0 && a<=size(M,2)
                for mpos = 1:size(M,2)
                    if a == mpos
                        transload=transload+17.77*10^-9+7.81 * 10^-9;
                    else
                        transload=transload+17.77*10^-9;
                    end
                end
                transload = transload / size(M,2);
            end
            y=[B(i).edges.pre] == j & [B(i).edges.suc] == x;
            eest_lt = ft + B(i).edges(y).trans * transload - LT(i,x,B);
        end
        if t<eest_lt
            t=eest_lt;
        end
        if eest_lt>0
            flag=-1;
        end
    end
end

function t=readytime(i,j,B)
    suc = findsuc(B,i,j);
    t=0;
    for x = suc
        if t < B(i).nodes(j+1).ft
            t=B(i).nodes(j+1).ft;
        end
    end
end

function B=resort2(B)  %sort
    for i = 1:size(B,2)
        for j = size(B(i).nodes,1)-1 : -1 : 0
            B(i).nodes(j+1).rank = round(rerank2(B,i,j),3);
        end
    end
end
function rank=rerank2(B,i,a)  %Calculate the rank for task a in task graph i
    if a == size(B(i).nodes,1)-1
        rank = 0;
    else
        suc = findsuc(B,i,a);
        rank = 0;
        for m = suc
            j=[B(i).edges.pre] == a & [B(i).edges.suc] == m;
            x=B(i).nodes(m+1).rank + B(i).nodes(m+1).comp/5000 + B(i).edges(j).trans*17.77*10^-9;
            if rank < x
                rank = x;
            end
        end
    end
end
