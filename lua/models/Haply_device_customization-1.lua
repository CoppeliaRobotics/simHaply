sim = require 'sim'
simHaply = require 'simHaply'

function sysCall_init()
end

function sysCall_userConfig()
    local self = sim.getObject '.'
    simUI = require 'simUI'
    if ui then
        simUI.destroy(ui)
        ui = nil
    else
        ui = simUI.create[[<ui layout="grid" closeable="true" resizable="true" title="Haply device selection" on-close="onClose">
            <label text="Inverse3:" />
            <br />
            <combobox id="${ui_inv3_combo}" on-change="onComboboxInv3Change" />
            <button id="${ui_inv3_rescan}" text="Rescan" on-click="onRescanInv3Clicked" />
            <br />
            <label text="Handle:" />
            <br />
            <combobox id="${ui_hndl_combo}" on-change="onComboboxHndlChange" />
            <button id="${ui_hndl_rescan}" text="Rescan" on-click="onRescanHndlClicked" />
            <br />
        </ui>]]

        local inv3_ports = sim.getTableProperty(self, 'customData.haply_inverse3_ports', {noError = true})
        if not inv3_ports then
            inv3_ports = onRescanInv3Clicked(ui, ui_inv3_rescan)
        end
        simUI.setComboboxItems(ui, ui_inv3_combo, inv3_ports, -1)
        local inv3_port = sim.getStringProperty(self, 'customData.haply_inverse3_port', {noError = true})
        if inv3_port then
            local i = table.find(inv3_ports, inv3_port)
            if i then
                simUI.setComboboxSelectedIndex(ui, ui_inv3_combo, i - 1)
            end
        end

        local hndl_ports = sim.getTableProperty(self, 'customData.haply_handle_ports', {noError = true})
        if not hndl_ports then
            hndl_ports = onRescanHndlClicked(ui, ui_hndl_rescan)
        end
        simUI.setComboboxItems(ui, ui_hndl_combo, hndl_ports, -1)
        local hndl_port = sim.getStringProperty(self, 'customData.haply_handle_port', {noError = true})
        if hndl_port then
            local i = table.find(hndl_ports, hndl_port)
            if i then
                simUI.setComboboxSelectedIndex(ui, ui_hndl_combo, i - 1)
            end
        end
    end
end

function onComboboxInv3Change(ui, id, newIdx)
    local self = sim.getObject '.'
    local ports = sim.getTableProperty(self, 'customData.haply_inverse3_ports', {noError = true})
    local port = nil
    if newIdx >= 0 then
        port = ports[1 + newIdx]
    end
    sim.setStringProperty(self, 'customData.haply_inverse3_port', port)
end

function onComboboxHndlChange(ui, id, newIdx)
    local self = sim.getObject '.'
    local ports = sim.getTableProperty(self, 'customData.haply_handle_ports', {noError = true})
    local port = nil
    if newIdx >= 0 then
        port = ports[1 + newIdx]
    end
    sim.setStringProperty(self, 'customData.haply_handle_port', port)
end

function onRescanInv3Clicked(ui, id)
    local self = sim.getObject '.'
    local oldIndex = simUI.getComboboxSelectedIndex(ui, ui_inv3_combo)
    simUI.setComboboxItems(ui, ui_inv3_combo, {'Scanning...'}, 0)
    simUI.setEnabled(ui, ui_inv3_combo, false)
    simUI.setEnabled(ui, ui_inv3_rescan, false)
    local ports = simHaply.detectInverse3s()
    local port = sim.getStringProperty(self, 'customData.haply_inverse3_port', {noError = true})
    if port then
        local i = table.find(ports, port)
        if i then oldIndex = i - 1 end
    end
    simUI.setComboboxItems(ui, ui_inv3_combo, ports, oldIndex)
    sim.setTableProperty(self, 'customData.haply_inverse3_ports', ports)
    simUI.setEnabled(ui, ui_inv3_combo, true)
    simUI.setEnabled(ui, ui_inv3_rescan, true)
    return ports
end

function onRescanHndlClicked(ui, id)
    local self = sim.getObject '.'
    local oldIndex = simUI.getComboboxSelectedIndex(ui, ui_hndl_combo)
    simUI.setComboboxItems(ui, ui_hndl_combo, {'Scanning...'}, 0)
    simUI.setEnabled(ui, ui_hndl_combo, false)
    simUI.setEnabled(ui, ui_hndl_rescan, false)
    local ports = simHaply.detectHandles()
    local port = sim.getStringProperty(self, 'customData.haply_handle_port', {noError = true})
    if port then
        local i = table.find(ports, port)
        if i then oldIndex = i - 1 end
    end
    simUI.setComboboxItems(ui, ui_hndl_combo, ports, oldIndex)
    sim.setTableProperty(self, 'customData.haply_handle_ports', ports)
    simUI.setEnabled(ui, ui_hndl_combo, true)
    simUI.setEnabled(ui, ui_hndl_rescan, true)
    return ports
end

function onClose()
    simUI.destroy(ui)
    ui = nil
end
